import time
from abc import abstractmethod

from pybricks.ev3devices import Motor, ColorSensor, UltrasonicSensor
from pybricks.hubs import EV3Brick
from pybricks.messaging import BluetoothMailboxClient, BluetoothMailboxServer, TextMailbox
from pybricks.parameters import Port
from pybricks.robotics import DriveBase
from pybricks.tools import wait

from constants import STOP_SIGN, MAILBOX_NAME, SERVER_VEHICLE_NAME, SCHOOL_ZONE_SIGN, LAB_END_SIGN, \
    OBSTACLE_DETECTED_SIGN, PARKING_ENDED, PARKING_LOT_DETECTED
from enums import Lane
from objectdetector import BlueColorDetector, ObstacleDetector, \
    SimpleRedColorDetector, SimpleYellowColorDetector, ParkingLotDetector
from strategy import ParallelParkingStrategy, ReversePerpendicularParkingStrategy


class VehicleFactory:
    class PlatooningConfiguration:
        PLATOONING_OFF = 0
        PLATOONING_LEADER = 1
        PLATOONING_FOLLOWER = 2

    drive_speed = 100
    proportional_gain = 0.8

    def __init__(self):
        # Initialize EV3 hardware components
        self._left_color_sensor = ColorSensor(Port.S1)
        self._right_color_sensor = ColorSensor(Port.S4)
        self._obstacle_sensor = UltrasonicSensor(Port.S3)
        self._parking_lot_sensor = UltrasonicSensor(Port.S2)

        self._motor_left = Motor(Port.B)
        self._motor_right = Motor(Port.C)

        self._ev3_brick = EV3Brick()
        self._drive_base = DriveBase(self._motor_left, self._motor_right, wheel_diameter=55.5, axle_track=104)

        self._parking_strategy = ParallelParkingStrategy(self)

        # Set vehicle's initial states
        self.current_lane = Lane.FIRST_LANE
        self.lab_finished = False
        self.first_parking_lot_indicator_detected = False

        self.parking_lot_detector = ParkingLotDetector(queue_len=3, threshold=2, ultrasonic_sensor=self._parking_lot_sensor, enabled=False)

    @classmethod
    def create_vehicle(cls, platooning_mode=PlatooningConfiguration.PLATOONING_OFF):
        vehicle_class_dict = {
            cls.PlatooningConfiguration.PLATOONING_OFF: StandaloneVehicle,
            cls.PlatooningConfiguration.PLATOONING_LEADER: PlatooningLeaderVehicle,
            cls.PlatooningConfiguration.PLATOONING_FOLLOWER: PlatooningFollowerVehicle,
        }
        try:
            return vehicle_class_dict[platooning_mode]()
        except KeyError:
            raise Exception('platooning_mode not defined')

    def start_driving(self):
        self._beep()

        while True:
            if self.detect_parking_vehicle():
                self.wait_for_parking()
            # TODO: Implement pre-driving method using detector and handler registry
            if self.detect_pause_block():
                self.pause()

            if self.detect_school_zone_block():
                self.slow_down_vehicle()

            if self.detect_obstacle():
                self.change_lane()

            if self.detect_lab_end_block():
                self.set_lab_finished()

            if self.detect_parking_lot():
                self.start_parking()
                return

            self.drive(self.drive_speed)

    def detect_parking_vehicle(self):
        return False

    def wait_for_parking(self):
        pass

    def set_lab_finished(self):
        self._beep()
        self.lab_finished = True
        self.parking_lot_detector.enable()

    def pause(self):
        pause_duration_milliseconds = 3000

        self._beep()
        self._stop()
        wait(pause_duration_milliseconds)
        self._beep()

    def slow_down_vehicle(self):
        slow_drive_speed = self.drive_speed / 2
        slow_drive_duration_seconds = 2.5
        slow_down_start_time = time.time()

        self._beep()
        while time.time() - slow_down_start_time <= slow_drive_duration_seconds:
            self.drive(slow_drive_speed)
        self._beep()

    def change_lane(self):
        turn_duration_seconds = 0.8
        return_duration_seconds = 0.8
        proportional_gain = 0.13

        # Decide turn direction
        if self.current_lane == Lane.SECOND_LANE:
            proportional_gain *= -1

        # Set initial turn rate value
        turn_rate = self._get_turn_rate()

        turn_start_time = time.time()
        while time.time() - turn_start_time <= turn_duration_seconds:
            turn_rate += proportional_gain
            self._drive_base.drive(self.drive_speed, turn_rate)

        turn_start_time = time.time()
        while time.time() - turn_start_time <= return_duration_seconds * 2:
            turn_rate -= proportional_gain
            self._drive_base.drive(self.drive_speed, turn_rate)

        weight = 0
        weight_increment_amount = 0.03
        turn_start_time = time.time()
        while time.time() - turn_start_time <= turn_duration_seconds:
            turn_rate += proportional_gain

            temp_turn_rate = self._get_turn_rate()

            if weight + weight_increment_amount < 1:
                weight += weight_increment_amount

            total_turn_rate = turn_rate * (1 - weight) + temp_turn_rate * weight

            self._drive_base.drive(self.drive_speed, total_turn_rate)

        self.current_lane = Lane.SECOND_LANE if self.current_lane == Lane.FIRST_LANE else Lane.FIRST_LANE

    @abstractmethod
    def detect_pause_block(self):
        pass

    @abstractmethod
    def detect_school_zone_block(self):
        pass

    @abstractmethod
    def detect_lab_end_block(self):
        pass

    @abstractmethod
    def detect_obstacle(self):
        pass

    @abstractmethod
    def detect_parking_lot(self):
        pass

    def drive(self, drive_speed, turn_rate=None):
        if turn_rate is None:
            turn_rate = self._get_turn_rate()

        self._drive_base.drive(
            speed=drive_speed,
            turn_rate=turn_rate
        )

    def start_parking(self):
        self._parking_strategy.start_parking()

    def _get_turn_rate(self):
        deviation = self._left_color_sensor.reflection() - self._right_color_sensor.reflection()

        return self.proportional_gain * deviation

    def _beep(self):
        self._ev3_brick.speaker.beep()

    def _stop(self):
        self._drive_base.stop()


class StandaloneVehicle(VehicleFactory):
    def __init__(self):
        super(StandaloneVehicle, self).__init__()

        color_sensor_list = [self._left_color_sensor, self._right_color_sensor]
        self.yellow_color_detector = SimpleYellowColorDetector(queue_len=4, threshold=3, color_sensor_list=color_sensor_list)
        self.red_color_detector = SimpleRedColorDetector(queue_len=4, threshold=3, color_sensor_list=color_sensor_list)
        self.blue_color_detector = BlueColorDetector(queue_len=2, threshold=1, color_sensor_list=color_sensor_list)
        self.obstacle_detector = ObstacleDetector(queue_len=6, threshold=3, ultrasonic_sensor=self._obstacle_sensor)

    def detect_pause_block(self):
        return self.blue_color_detector.detected()

    def detect_school_zone_block(self):
        return self.yellow_color_detector.detected()

    def detect_lab_end_block(self):
        return self.red_color_detector.detected()

    def detect_obstacle(self):
        return self.obstacle_detector.detected()

    def detect_parking_lot(self):
        if self.parking_lot_detector.detected():
            if self.first_parking_lot_indicator_detected:
                self._beep()
                return True
            else:
                self._beep()
                self.first_parking_lot_indicator_detected = True
                return False
        else:
            return False


class PlatooningLeaderVehicle(StandaloneVehicle):
    def __init__(self):
        super(PlatooningLeaderVehicle, self).__init__()

        self.server = BluetoothMailboxServer()
        self.mbox = TextMailbox(MAILBOX_NAME, self.server)

        # The server must be started before the client!
        print('waiting for connection...')
        self.server.wait_for_connection()
        print('connected!')

        self._parking_strategy = ReversePerpendicularParkingStrategy(self)

    def _send_message(self, value: str):
        message = "{}:{}".format(value, str(int(time.time())))
        print(message)
        self.mbox.send(message)

    def set_lab_finished(self):
        self._send_message(LAB_END_SIGN)
        super(PlatooningLeaderVehicle, self).set_lab_finished()

    def pause(self):
        self._send_message(STOP_SIGN)
        super(PlatooningLeaderVehicle, self).pause()

    def slow_down_vehicle(self):
        self._send_message(SCHOOL_ZONE_SIGN)
        super(PlatooningLeaderVehicle, self).slow_down_vehicle()

    def change_lane(self):
        self._send_message(OBSTACLE_DETECTED_SIGN)
        super(PlatooningLeaderVehicle, self).change_lane()

    def detect_parking_lot(self):
        if self.parking_lot_detector.detected():
            if self.first_parking_lot_indicator_detected:
                self._beep()
                # TODO: Move signaling logic to Separate pre_parking method
                return True
            else:
                self._beep()
                self._send_message(PARKING_LOT_DETECTED)
                self.first_parking_lot_indicator_detected = True
                return False
        else:
            return False

    def start_parking(self):
        super(PlatooningLeaderVehicle, self).start_parking()
        self._send_message(PARKING_ENDED)


class PlatooningFollowerVehicle(VehicleFactory):
    def __init__(self):
        super(PlatooningFollowerVehicle, self).__init__()

        self.client = BluetoothMailboxClient()
        self.mbox = TextMailbox(MAILBOX_NAME, self.client)

        print('establishing connection...')
        self.client.connect(SERVER_VEHICLE_NAME)
        print('connected!')

        self.parking_lot_detector = ObstacleDetector(queue_len=3, threshold=2, ultrasonic_sensor=self._parking_lot_sensor)

        self._parking_strategy = ReversePerpendicularParkingStrategy(self)

        # TODO: Improve block detection mechanism to use timestamp comparison
        self.stop_signal_detected = False
        self.school_zone_signal_detected = False

        self.obstacle_message_list = []

    def detect_pause_block(self):
        current_message = self.mbox.read()
        if current_message is None:
            return False
        try:
            message_name, _ = current_message.split(":")
        except ValueError:
            return False

        # TODO: Improve to use timestamp to separate detection
        if message_name == STOP_SIGN and not self.stop_signal_detected:
            self.stop_signal_detected = True
            return True
        else:
            return False

    def detect_school_zone_block(self):
        current_message = self.mbox.read()
        if current_message is None:
            return False
        try:
            message_name, _ = current_message.split(":")
        except ValueError:
            return False

        # TODO: Improve to use timestamp to separate detection
        if message_name == SCHOOL_ZONE_SIGN and not self.school_zone_signal_detected:
            self.school_zone_signal_detected = True
            return True
        else:
            return False

    def detect_lab_end_block(self):
        current_message = self.mbox.read()
        if current_message is None:
            return False
        try:
            message_name, _ = current_message.split(":")
        except ValueError:
            return False

        # TODO: Improve to use timestamp to separate detection
        if message_name == LAB_END_SIGN and not self.school_zone_signal_detected:
            self.school_zone_signal_detected = True
            return True
        else:
            return False

    def detect_obstacle(self):
        current_message = self.mbox.read()
        if current_message is None:
            return False
        try:
            message_name, sent_at = current_message.split(":")
        except ValueError:
            return False

        if message_name == OBSTACLE_DETECTED_SIGN:
            if sent_at in self.obstacle_message_list:
                return False
            else:
                self.obstacle_message_list.append(sent_at)
                return True
        else:
            return False

    def detect_parking_lot(self):
        # TODO: Resolve code duplicates
        if self.parking_lot_detector.detected():
            if self.first_parking_lot_indicator_detected:
                return True
            else:
                self.first_parking_lot_indicator_detected = True
                return False
        else:
            return False

    def drive(self, drive_speed, turn_rate=None):
        weight = 0.5
        objective_distance_millimeter = 200
        max_distance_millimeter = 240

        current_distance_millimeter = self._obstacle_sensor.distance()
        if current_distance_millimeter > max_distance_millimeter:
            current_distance_millimeter = max_distance_millimeter

        bias = current_distance_millimeter - objective_distance_millimeter
        drive_speed += weight * bias

        if drive_speed < 0:
            drive_speed = 0

        super(PlatooningFollowerVehicle, self).drive(drive_speed=drive_speed, turn_rate=turn_rate)

    def detect_parking_vehicle(self):
        # TODO: Extract method and handle exception
        current_message = self.mbox.read()
        if current_message is None:
            return False
        try:
            message_name, _ = current_message.split(":")
        except ValueError:
            return False

        # TODO: Improve to use timestamp to separate detection
        if message_name == SCHOOL_ZONE_SIGN:
            pass

import time
from abc import abstractmethod

from pybricks.ev3devices import Motor, ColorSensor, UltrasonicSensor
from pybricks.hubs import EV3Brick
from pybricks.messaging import BluetoothMailboxClient, BluetoothMailboxServer, TextMailbox
from pybricks.parameters import Port
from pybricks.robotics import DriveBase
from pybricks.tools import wait

from constants import MAILBOX_NAME, SERVER_VEHICLE_NAME
from enums import Lane
from logger import EventLogger, ColorSensorLogger
from objectdetector import BlueColorDetector, ObstacleDetector, \
    SimpleRedColorDetector, ParkingLotDetector, YellowColorDetector
from strategy import ParallelParkingStrategy, ReversePerpendicularParkingStrategy
from utility import PlatooningMessage, WrongMessageFormat, PlatooningMessageRegistry


class VehicleFactory:
    class PlatooningConfiguration:
        PLATOONING_OFF = 0
        PLATOONING_LEADER = 1
        PLATOONING_FOLLOWER = 2

    drive_speed = 100
    proportional_gain = 0.8

    def __init__(self):
        self.logger = EventLogger()
        self.data_logger = ColorSensorLogger()

        # Initialize EV3 hardware components
        self._left_color_sensor = ColorSensor(Port.S1)
        self._right_color_sensor = ColorSensor(Port.S4)
        self._obstacle_sensor = UltrasonicSensor(Port.S3)
        self._parking_lot_sensor = UltrasonicSensor(Port.S2)

        self._motor_left = Motor(Port.B)
        self._motor_right = Motor(Port.C)

        self._ev3_brick = EV3Brick()
        self._drive_base = DriveBase(self._motor_left, self._motor_right, wheel_diameter=55.5, axle_track=104)
        self.parking_lot_detector = ParkingLotDetector(queue_len=2, threshold=1,
                                                       ultrasonic_sensor=self._parking_lot_sensor, enabled=False)

        self._parking_strategy = ParallelParkingStrategy(self)

        # Set vehicle's initial states
        self.current_lane = Lane.FIRST_LANE
        self.first_parking_lot_indicator_detected = False

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
        self.logger.info("Started Driving")
        self._beep()

        while True:
            self.data_logger.log(
                self._left_color_sensor.color(),
                self._left_color_sensor.rgb(),
                self._right_color_sensor.color(),
                self._right_color_sensor.rgb()
            )

            if self.detect_parking_vehicle():
                self.wait_for_parking()

            if self.detect_pause_block():
                self.logger.info("Pause Block Detected")
                self.pause()

            if self.detect_school_zone_block():
                self.logger.info("School Zone Block Detected")
                self.slow_down_vehicle()

            if self.detect_obstacle():
                self.logger.info("Obstacle Detected")
                self.change_lane()

            if self.detect_lab_end_block():
                self.logger.info("Lab End Block Detected")
                self.set_lab_finished()

            if self.detect_parking_lot():
                self.logger.info("Parking Lot Detected")
                self.start_parking()
                break

            self.drive(self.drive_speed)

        self.logger.info("Finished Driving")

    def detect_parking_vehicle(self):
        return False

    def wait_for_parking(self):
        pass

    def set_lab_finished(self):
        self._beep()
        self.parking_lot_detector.enable()

    def pause(self):
        self.logger.info("Paused Driving")
        pause_duration_milliseconds = 3000

        self._beep()
        self._stop()
        wait(pause_duration_milliseconds)
        self._beep()
        self.logger.info("Resume Driving")

    def slow_down_vehicle(self):
        self.logger.info("Start Driving at Half Speed")
        slow_drive_speed = self.drive_speed / 2
        slow_drive_duration_seconds = 2.5
        slow_down_start_time = time.time()

        self._beep()
        while time.time() - slow_down_start_time <= slow_drive_duration_seconds:
            self.drive(slow_drive_speed)
        self._beep()
        self.logger.info("Resume Driving")

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

    def detect_parking_lot(self):
        if self.parking_lot_detector.detected():
            if self.first_parking_lot_indicator_detected:
                self.logger.info("Second Parking Log Indicator Detected")
                self._beep()
                return True
            else:
                self.logger.info("First Parking Log Indicator Detected")
                self._beep()
                self.first_parking_lot_indicator_detected = True
                return False
        else:
            return False

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
        self.obstacle_detector = ObstacleDetector(queue_len=6, threshold=3, ultrasonic_sensor=self._obstacle_sensor)

        self.red_color_detector = SimpleRedColorDetector(queue_len=4, threshold=3, color_sensor_list=color_sensor_list)
        self.yellow_color_detector = YellowColorDetector(queue_len=3, threshold=2, color_sensor_list=color_sensor_list)
        self.blue_color_detector = BlueColorDetector(queue_len=3, threshold=2, color_sensor_list=color_sensor_list)

    def detect_pause_block(self):
        return self.blue_color_detector.detected()

    def detect_school_zone_block(self):
        return self.yellow_color_detector.detected()

    def detect_lab_end_block(self):
        return self.red_color_detector.detected()

    def detect_obstacle(self):
        return self.obstacle_detector.detected()


class PlatooningLeaderVehicle(StandaloneVehicle):
    def __init__(self):
        super(PlatooningLeaderVehicle, self).__init__()

        self.server = BluetoothMailboxServer()
        self.mbox = TextMailbox(MAILBOX_NAME, self.server)

        # The server must be started before the client!
        self.logger.info('waiting for connection...')
        self.server.wait_for_connection()
        self.logger.info('connected!')

        self._parking_strategy = ReversePerpendicularParkingStrategy(self)

    def detect_parking_lot(self):
        # TODO: Improve to use upper class method with signaling
        if self.parking_lot_detector.detected():
            if self.first_parking_lot_indicator_detected:
                self.logger.info("Second Parking Log Indicator Detected")
                self._beep()
                return True
            else:
                self._beep()
                self._send_message(PlatooningMessage.PARKING_LOT_DETECTED)
                self.first_parking_lot_indicator_detected = True
                self.logger.info("First Parking Log Indicator Detected")
                return False
        else:
            return False

    def pause(self):
        self._send_message(PlatooningMessage.STOP_SIGN)
        super(PlatooningLeaderVehicle, self).pause()

    def slow_down_vehicle(self):
        self._send_message(PlatooningMessage.SCHOOL_ZONE_SIGN)
        super(PlatooningLeaderVehicle, self).slow_down_vehicle()

    def change_lane(self):
        self._send_message(PlatooningMessage.OBSTACLE_DETECTED_SIGN)
        super(PlatooningLeaderVehicle, self).change_lane()

    def set_lab_finished(self):
        self._send_message(PlatooningMessage.LAB_END_SIGN)
        super(PlatooningLeaderVehicle, self).set_lab_finished()

    def start_parking(self):
        self._send_message(PlatooningMessage.PARKING_ENDED)
        super(PlatooningLeaderVehicle, self).start_parking()

    def _send_message(self, value: str):
        message = PlatooningMessage.from_data(value)
        self.mbox.send(str(message))
        self.logger.info("Message sent: '{}'".format(str(message)))


class PlatooningFollowerVehicle(VehicleFactory):
    def __init__(self):
        super(PlatooningFollowerVehicle, self).__init__()

        self._client = BluetoothMailboxClient()
        self._mbox = TextMailbox(MAILBOX_NAME, self._client)

        self.logger.info('establishing connection...')
        self._client.connect(SERVER_VEHICLE_NAME)
        self.logger.info('connected!')

        self._parking_strategy = ReversePerpendicularParkingStrategy(self)
        self._message_registry = PlatooningMessageRegistry()

    def detect_pause_block(self):
        return self._check_message(PlatooningMessage.STOP_SIGN)

    def detect_school_zone_block(self):
        return self._check_message(PlatooningMessage.SCHOOL_ZONE_SIGN)

    def detect_lab_end_block(self):
        return self._check_message(PlatooningMessage.LAB_END_SIGN)

    def detect_obstacle(self):
        return self._check_message(PlatooningMessage.OBSTACLE_DETECTED_SIGN)

    def detect_parking_vehicle(self):
        return self._check_message(PlatooningMessage.PARKING_LOT_DETECTED)

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

    def change_lane(self):
        delay_duration_milliseconds = 1500

        self._stop()
        wait(delay_duration_milliseconds)
        super(PlatooningFollowerVehicle, self).change_lane()

    def wait_for_parking(self):
        self._beep()
        self._stop()

        while True:
            current_message = self._mbox.wait_new()
            try:
                message_name, _ = current_message.split(":")
            except ValueError:
                self.logger.error("Badly formatted message received: '{}'".format(current_message))
                return False

            if message_name == PlatooningMessage.PARKING_ENDED:
                self.logger.info("PARKING_ENDED message received: " + current_message)
                break

        self._beep()

    def _check_message(self, code):
        message = self._read_message()

        if message is None or message.code != code or self._message_registry.find(message):
            return False
        else:
            self.logger.info("{} message received ({})".format(code, str(message)))
            self._message_registry.insert(message)
            return True

    def _read_message(self):
        current_message_data = self._mbox.read()
        if current_message_data is None:
            return None

        message = None

        try:
            message = PlatooningMessage.from_data(current_message_data)
        except WrongMessageFormat:
            self.logger.warning("Bad Formatted Message Received: " + current_message_data)

        return message

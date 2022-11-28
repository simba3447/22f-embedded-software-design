from abc import abstractmethod

from pybricks.ev3devices import Motor, ColorSensor, UltrasonicSensor
from pybricks.hubs import EV3Brick
from pybricks.parameters import Port
from pybricks.robotics import DriveBase
from pybricks.tools import wait

from enums import Lane
from objectdetector import RedColorDetector, BlueColorDetector, YellowColorDetector, ObstacleDetector

import time


class NexGenVehicleFactory:
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
        self._distance_sensor = UltrasonicSensor(Port.S3)
        self._park_sensor = UltrasonicSensor(Port.S2)

        self._motor_left = Motor(Port.B)
        self._motor_right = Motor(Port.C)

        self._ev3_brick = EV3Brick()
        self._drive_base = DriveBase(self._motor_left, self._motor_right, wheel_diameter=55.5, axle_track=104)

        # Set vehicle's initial states
        self.current_lane = Lane.FIRST_LANE
        self.lab_finished = False

    @classmethod
    def create_vehicle(cls, platooning_mode = PlatooningConfiguration.PLATOONING_OFF):
        vehicle_class_dict = {
            cls.PlatooningConfiguration.PLATOONING_OFF: StandaloneVehicle,
            cls.PlatooningConfiguration.PLATOONING_LEADER: PlatooningLeaderVehicle,
            cls.PlatooningConfiguration.PLATOONING_FOLLOWER: PlatooningFollowerVehicle,
        }
        try:
            return vehicle_class_dict[platooning_mode]()
        except KeyError:
            raise Exception('platooning_mode not defined')

    def drive(self):
        self._beep()

        while True:
            if self.detect_pause_block():
                self.pause()

            if self.detect_school_zone_block():
                self.slow_down_vehicle()

            if self.detect_obstacle():
                self.change_lane()

            if self.detect_lab_end_block():
                self._beep()
                return

            self._drive(self.drive_speed)

    def pause(self):
        pause_duration_milliseconds = 3000

        self._beep()
        self._stop()
        wait(pause_duration_milliseconds)
        self._beep()

    def slow_down_vehicle(self):
        slow_drive_duration_seconds = 2.5
        slow_down_start_time = time.time()

        self._beep()
        while time.time() - slow_down_start_time <= slow_drive_duration_seconds:
            self._drive(self.drive_speed / 2)
        self._beep()

    def change_lane(self):
        # TODO: Refactor code
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

    def _drive(self, drive_speed):
        self._drive_base.drive(
            speed=drive_speed,
            turn_rate=self._get_turn_rate()
        )

    def _get_turn_rate(self):
        deviation = self._left_color_sensor.reflection() - self._right_color_sensor.reflection()

        return self.proportional_gain * deviation

    def _beep(self):
        self._ev3_brick.speaker.beep()

    def _stop(self):
        self._drive_base.stop()


class StandaloneVehicle(NexGenVehicleFactory):
    def __init__(self):
        super(StandaloneVehicle, self).__init__()

        color_sensor_list = [self._left_color_sensor, self._right_color_sensor]
        self.blue_color_detector = BlueColorDetector(queue_len=2, threshold=1, color_sensor_list=color_sensor_list)
        self.yellow_color_detector = YellowColorDetector(queue_len=2, threshold=1, color_sensor_list=color_sensor_list)
        self.red_color_detector = RedColorDetector(queue_len=2, threshold=1, color_sensor_list=color_sensor_list)
        self.obstacle_detector = ObstacleDetector(queue_len=6, threshold=3, ultrasonic_sensor=self._distance_sensor)

    def detect_pause_block(self):
        return self.blue_color_detector.detected()

    def detect_school_zone_block(self):
        return self.yellow_color_detector.detected()

    def detect_lab_end_block(self):
        return self.red_color_detector.detected()

    def detect_obstacle(self):
        return self.obstacle_detector.detected()


class PlatooningLeaderVehicle(StandaloneVehicle):
    def pause(self):
        # TODO: send pause signal to follower vehicle
        super(PlatooningLeaderVehicle, self).pause()

    def slow_down_vehicle(self):
        # TODO: send slow down signal to follower vehicle
        super(PlatooningLeaderVehicle, self).slow_down_vehicle()


class PlatooningFollowerVehicle(NexGenVehicleFactory):
    def detect_pause_block(self):
        # TODO: detect pause block using message from server vehicle
        pass

    def detect_school_zone_block(self):
        # TODO: detect school zone block using message from server vehicle
        pass

    def detect_lab_end_block(self):
        # TODO: detect lab end block using message from server vehicle
        pass

    def detect_obstacle(self):
        # TODO: detect obstacle using message from server vehicle
        pass

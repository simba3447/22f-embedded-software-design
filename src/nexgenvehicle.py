from pybricks.ev3devices import Motor, ColorSensor, UltrasonicSensor
from pybricks.hubs import EV3Brick
from pybricks.parameters import Port
from pybricks.robotics import DriveBase
from pybricks.tools import wait

from enums import Lane
from objectdetector import RedColorDetector, BlueColorDetector, YellowColorDetector

import time

class PlatooningConfiguration:
    PLATOONING_OFF = 0
    PLATOONING_LEADER = 1
    PLATOONING_FOLLOWER = 2

class NexGenVehicle:
    drive_speed = 100
    proportional_gain = 0.8

    def __init__(self, platooning_mode = PlatooningConfiguration.PLATOONING_OFF):
        self.platooning_mode = platooning_mode
        self.current_lane = Lane.FIRST_LANE

        self.left_color_sensor = ColorSensor(Port.S1)
        self.right_color_sensor = ColorSensor(Port.S4)
        self.distance_sensor = UltrasonicSensor(Port.S3)
        self.park_sensor = UltrasonicSensor(Port.S2)

        self.motor_left = Motor(Port.B)
        self.motor_right = Motor(Port.C)

        self.ev3_brick = EV3Brick()
        self.drive_base = DriveBase(self.motor_left, self.motor_right, wheel_diameter=55.5, axle_track=104)

        self.blue_color_detector = BlueColorDetector(queue_len=2, decision_criteria=1, color_sensor_list=[self.left_color_sensor, self.right_color_sensor])
        self.yellow_color_detector = YellowColorDetector(queue_len=2, decision_criteria=1, color_sensor_list=[self.left_color_sensor, self.right_color_sensor])
        self.red_color_detector = RedColorDetector(queue_len=2, decision_criteria=1, color_sensor_list=[self.left_color_sensor, self.right_color_sensor])

    def drive(self):
        self._beep()

        while True:
            if self.detect_pause_block():
                self.pause()

            if self.detect_school_zone_block():
                self.slow_down_vehicle()

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

    def detect_pause_block(self):
        return self.blue_color_detector.color_detected()

    def detect_school_zone_block(self):
        return self.yellow_color_detector.color_detected()

    def detect_lab_end_block(self):
        return self.red_color_detector.color_detected()

    def _drive(self, drive_speed):
        self.drive_base.drive(
            speed=drive_speed,
            turn_rate=self._get_turn_rate()
        )

    def _get_turn_rate(self):
        deviation = self.left_color_sensor.reflection() - self.right_color_sensor.reflection()

        return self.proportional_gain * deviation

    def _beep(self):
        self.ev3_brick.speaker.beep()

    def _stop(self):
        self.drive_base.stop()

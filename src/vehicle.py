from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, ColorSensor, UltrasonicSensor)
from pybricks.tools import wait
from pybricks.robotics import DriveBase

import time

from src.objectdetector import ObjectDetector

DRIVE_SPEED = 100
PROPORTIONAL_GAIN = 0.8
BREAK_BLOCK_DETECT_COUNT_THRESHOLD = 3
OBSTACLE_BLOCK_DETECT_COUNT_THRESHOLD = 3
PARK_BLOCK_DETECT_COUNT_THRESHOLD = 3

RGB_LOWER_BOUND = 30
RGB_UPPER_BOUND = 20

FIRST_LANE = 0
SECOND_LANE = 1


class Vehicle:
    def __init__(self, port_motor_l, port_motor_r, port_sensor_l, port_sensor_r, port_sensor_distance,
                 port_sensor_park):
        self.motor_l = Vehicle.initialize_motor(port_motor_l)
        self.motor_r = Vehicle.initialize_motor(port_motor_r)

        self.ev3 = EV3Brick()
        self.drive_base = DriveBase(self.motor_l, self.motor_r, wheel_diameter=55.5, axle_track=104)

        self.sensor_l = Vehicle.initialize_color_sensor(port_sensor_l)
        self.sensor_r = Vehicle.initialize_color_sensor(port_sensor_r)

        self.distance_sensor = UltrasonicSensor(port_sensor_distance)
        self.park_sensor = UltrasonicSensor(port_sensor_park)

        self.last_break_datetime = time.time()
        self.last_obstacle_datetime = time.time()
        self.lab_end_block_datetime = time.time()
        self.park_block_datetime = time.time()

        self.break_block_count = 0
        self.terminate_block_count = 0
        self.obstacle_block_count = 0
        self.park_block_count = 0

        self.obstacle_block_detector = ObjectDetector(queue_len=6, threshold=3)
        self.red_block_detector = ObjectDetector(queue_len=3, threshold=1)
        self.yellow_block_detector = ObjectDetector(queue_len=3, threshold=1)
        self.blue_block_detector = ObjectDetector(queue_len=3, threshold=1)

        self.current_lane = FIRST_LANE

        self.lab_finished = False
        self.detect_first_parking_sign = False

    @classmethod
    def initialize_motor(cls, port_motor):
        return Motor(port_motor)

    @classmethod
    def initialize_color_sensor(cls, port_sensor):
        return ColorSensor(port_sensor)

    @classmethod
    def detect_blue_color(cls, rgb_data):
        red, green, blue = rgb_data

        return red < RGB_UPPER_BOUND and green < RGB_UPPER_BOUND and blue > RGB_LOWER_BOUND

    @classmethod
    def detect_red_color(cls, rgb_data):
        red, green, blue = rgb_data

        return red > RGB_LOWER_BOUND and green < RGB_UPPER_BOUND and blue < RGB_UPPER_BOUND

    @classmethod
    def detect_yellow_color(cls, rgb_data):
        red, green, blue = rgb_data

        return red > RGB_LOWER_BOUND and green > RGB_LOWER_BOUND and blue < RGB_UPPER_BOUND

    def drive(self):
        self.ev3.speaker.beep()

        while True:
            turn_rate = self._get_turn_rate()

            self.pause_vehicle()
            self.change_lane()

            self.slow_down_vehicle()

            if Vehicle.detect_red_color(self.sensor_l.rgb()) or Vehicle.detect_red_color(self.sensor_r.rgb()):
                self.red_block_detector.add_detection_result(True)

                if not self.red_block_detector.detect_block():
                    pass
                if time.time() - self.lab_end_block_datetime < 3:
                    pass

                self.ev3.speaker.beep()
                self.lab_finished = True
                self.lab_end_block_datetime = time.time()

            else:
                self.red_block_detector.add_detection_result(False)

            if self.lab_finished and self.detect_parking():
                if time.time() - self.park_block_datetime < 2:
                    continue
                else:
                    if not self.detect_first_parking_sign:
                        self.detect_first_parking_sign = True
                        self.park_block_datetime = time.time()
                        continue
                    else:
                        self.ev3.speaker.beep()
                        self.start_parking()
                        self.drive_base.stop()
                        self.ev3.speaker.beep()
                        return

            self.drive_base.drive(DRIVE_SPEED, turn_rate)

    def _get_turn_rate(self):
        deviation = self.sensor_l.reflection() - self.sensor_r.reflection()
        return PROPORTIONAL_GAIN * deviation

    def pause_vehicle(self):
        if Vehicle.detect_blue_color(self.sensor_l.rgb()) or Vehicle.detect_blue_color(self.sensor_r.rgb()):
            self.blue_block_detector.add_detection_result(True)

            if time.time() - self.last_break_datetime < 2:
                return

            if not self.blue_block_detector._exceed_detection_threshold():
                return

            self.ev3.speaker.beep()
            self.drive_base.stop()
            wait(3000)

            self.last_break_datetime = time.time()
            self.break_block_count = 0
            self.blue_block_detector._reset_detection_result()
        else:
            self.blue_block_detector.add_detection_result(False)

    def slow_down_vehicle(self):
        slow_drive_speed = DRIVE_SPEED / 2
        slow_drive_duration = 2.5

        if Vehicle.detect_yellow_color(self.sensor_l.rgb()) or Vehicle.detect_yellow_color(self.sensor_r.rgb()):
            self.yellow_block_detector.add_detection_result(True)

            if not self.yellow_block_detector._exceed_detection_threshold():
                return

            if time.time() - self.last_break_datetime < 2:
                return

            self.ev3.speaker.beep()
            slow_down_start_time = time.time()

            while True:
                if time.time() - slow_down_start_time > slow_drive_duration:
                    break

                turn_rate = self._get_turn_rate()

                self.drive_base.drive(slow_drive_speed, turn_rate)
            self.last_break_datetime = time.time()
        else:
            self.yellow_block_detector.add_detection_result(False)

    def change_lane(self):
        obstacle_detect_distance = 250

        if int(self.distance_sensor.distance()) < obstacle_detect_distance:

            if not self.obstacle_block_detector._exceed_detection_threshold():
                self.obstacle_block_detector.add_detection_result(True)
                return

            self._change_lane()
        else:
            self.obstacle_block_detector.add_detection_result(False)

    def _change_lane(self):
        turn_duration_seconds = 0.8
        return_duration_seconds = 0.8
        proportional_gain = 0.13

        # Decide turn direction
        if self.current_lane == SECOND_LANE:
            proportional_gain *= -1

        turn_rate = self._get_turn_rate()

        turn_start_time = time.time()
        while True:
            if time.time() - turn_start_time > turn_duration_seconds:
                break
            turn_rate += proportional_gain
            self.drive_base.drive(DRIVE_SPEED, turn_rate)

        turn_start_time = time.time()
        while True:
            if time.time() - turn_start_time > return_duration_seconds * 2:
                break

            turn_rate -= proportional_gain

            self.drive_base.drive(DRIVE_SPEED, turn_rate)

        weight = 0
        weight_increment_amount = 0.03
        turn_start_time = time.time()
        while True:
            if time.time() - turn_start_time > turn_duration_seconds:
                break
            turn_rate += proportional_gain

            temp_turn_rate = self._get_turn_rate()

            if weight + weight_increment_amount < 1:
                weight += weight_increment_amount

            total_turn_rate = turn_rate * (1 - weight) + temp_turn_rate * weight

            self.drive_base.drive(DRIVE_SPEED, total_turn_rate)
        self.last_obstacle_datetime = time.time()
        self.obstacle_block_detector._reset_detection_result()

        # Change current lane state
        if self.current_lane == FIRST_LANE:
            self.current_lane = SECOND_LANE
        else:
            self.current_lane = FIRST_LANE

    def detect_parking(self):
        park_detect_distance_millimeter = 200
        if int(self.park_sensor.distance()) < park_detect_distance_millimeter:
            if self.park_block_count < OBSTACLE_BLOCK_DETECT_COUNT_THRESHOLD:
                self.park_block_count += 1
                return False
            return True
        return False

    def start_parking(self):
        reverse_drive_speed = DRIVE_SPEED * -0.5

        wait(500)

        turn_rate = 0

        straight_start_time = time.time()
        while True:
            if time.time() - straight_start_time > 1.5:
                break
            self.drive_base.drive(reverse_drive_speed, turn_rate)

        turn_start_time = time.time()

        while True:
            if time.time() - turn_start_time > 1.5:
                break
            turn_rate = -50

            self.drive_base.drive(reverse_drive_speed, turn_rate)

        wait(500)

        turn_rate = 0
        straight_start_time = time.time()
        while True:
            if time.time() - straight_start_time > 0.75:
                break
            self.drive_base.drive(reverse_drive_speed, turn_rate)

        wait(1000)

        turn_start_time = time.time()
        while True:
            if time.time() - turn_start_time > 2:
                break
            turn_rate = 50

            self.drive_base.drive(reverse_drive_speed, turn_rate)

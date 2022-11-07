#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

import time


# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.

# Create your objects here.
ev3 = EV3Brick()

# Write your program here.
ev3.speaker.beep()

motor_l = Motor(Port.B)
motor_r = Motor(Port.C)

robot = DriveBase(motor_l, motor_r, wheel_diameter=55.5, axle_track=104)

sensor_left = ColorSensor(Port.S1)
sensor_right = ColorSensor(Port.S4)

DRIVE_SPEED = 70
PROPORTIONAL_GAIN = 1.2
BREAK_BLOCK_DETECT_COUNT_THRESHOLD = 3

last_break_datetime = time.time()

break_block_count = 0
terminate_block_count = 0

def detect_green_color(rgb_data):
    LOWER_BOUND = 30
    UPPER_BOUND = 20
    red, green, blue = rgb_data
    print(str(red) + ", " + str(green) + ", " + str(blue))
    return red < UPPER_BOUND and blue < UPPER_BOUND and green > LOWER_BOUND


def detect_red_color(rgb_data):
    LOWER_BOUND = 30
    UPPER_BOUND = 20
    red, green, blue = rgb_data
    print(str(red) + ", " + str(green) + ", " + str(blue))
    return green < UPPER_BOUND and blue < UPPER_BOUND and red > LOWER_BOUND

while True:
    deviation = sensor_left.reflection() - sensor_right.reflection()
    turn_rate = PROPORTIONAL_GAIN * deviation

    robot.drive(DRIVE_SPEED, turn_rate)
    
    sensor_left_color = sensor_left.color()
    sensor_right_color = sensor_right.color()

    if (detect_red_color(sensor_left.rgb()) or detect_red_color(sensor_right.rgb())):
        if terminate_block_count < BREAK_BLOCK_DETECT_COUNT_THRESHOLD:
            terminate_block_count += 1
            pass

        print("!!! STOP!!! " + str(sensor_left_color)+ ", " + str(sensor_right_color))
        robot.stop()
        ev3.speaker.beep()
        break

    # wait(10)
    if (detect_green_color(sensor_left.rgb()) or detect_green_color(sensor_right.rgb())):
        if time.time() - last_break_datetime < 2:
            pass

        if break_block_count < BREAK_BLOCK_DETECT_COUNT_THRESHOLD:
            break_block_count += 1
            pass
        
        print("!!! BREAK!!! " + str(sensor_left_color)+ ", " + str(sensor_right_color))
        robot.stop()
        wait(3000)
        print("!!! CONTINUE!!!")
        last_break_datetime = time.time()
        brek_block_count = 0
        robot.drive(DRIVE_SPEED, turn_rate)

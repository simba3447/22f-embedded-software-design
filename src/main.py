#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
from vehicle import Vehicle

vehicle = Vehicle(
    port_motor_l=Port.B, 
    port_motor_r=Port.C, 
    port_sensor_l=Port.S1, 
    port_sensor_r=Port.S4,
    port_sensor_distance=Port.S3,
    port_sensor_park=Port.S2,
)
vehicle.drive()
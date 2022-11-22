#!/usr/bin/env pybricks-micropython
from pybricks.parameters import Port

from nexgenvehicle import NexGenVehicle
# from vehicle import Vehicle

# vehicle = Vehicle(
#     port_motor_l=Port.B,
#     port_motor_r=Port.C,
#     port_sensor_l=Port.S1,
#     port_sensor_r=Port.S4,
#     port_sensor_distance=Port.S3,
#     port_sensor_park=Port.S2,
# )
vehicle = NexGenVehicle()
vehicle.drive()

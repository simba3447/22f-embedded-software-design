#!/usr/bin/env pybricks-micropython
from vehicle import VehicleFactory

vehicle = VehicleFactory.create_vehicle(
    platooning_mode=VehicleFactory.PlatooningConfiguration.PLATOONING_OFF
)
vehicle.start_driving()

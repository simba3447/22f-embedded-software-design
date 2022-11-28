#!/usr/bin/env pybricks-micropython
from pybricks.parameters import Port

from nexgenvehicle import NexGenVehicleFactory

vehicle = NexGenVehicleFactory.create_vehicle(
    platooning_mode=NexGenVehicleFactory.PlatooningConfiguration.PLATOONING_OFF
)
vehicle.start_driving()

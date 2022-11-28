from abc import ABC, abstractmethod
from pybricks.tools import wait

import time

from src.nexgenvehicle import NexGenVehicleFactory


class ParkingStrategy(ABC):
    def __init__(self, vehicle: NexGenVehicleFactory):
        self.vehicle = vehicle

    @abstractmethod
    def start_parking(self):
        pass


class ParallelParkingStrategy(ParkingStrategy):
    def start_parking(self):
        reverse_drive_speed = self.vehicle.drive_speed * -0.5

        wait(500)

        straight_start_time = time.time()
        while time.time() - straight_start_time <= 1.5:
            self.vehicle.drive(drive_speed=reverse_drive_speed, turn_rate=0)

        turn_start_time = time.time()
        while time.time() - turn_start_time <= 1.5:
            self.vehicle.drive(drive_speed=reverse_drive_speed, turn_rate=-50)

        wait(500)

        straight_start_time = time.time()
        while time.time() - straight_start_time <= 0.75:
            self.vehicle.drive(drive_speed=reverse_drive_speed, turn_rate=0)

        wait(1000)

        turn_start_time = time.time()
        while time.time() - turn_start_time <= 2:
            self.vehicle.drive(drive_speed=reverse_drive_speed, turn_rate=50)


class ReversePerpendicularParkingStrategy(ParkingStrategy):
    def start_parking(self):
        # TODO: Implement parking method
        pass

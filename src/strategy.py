import time
from abc import abstractmethod

from pybricks.tools import wait


class ParkingStrategy:
    def __init__(self, vehicle):
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
        # TODO: Fix to use public method of vehicle for driving
        reverse_drive_speed = self.vehicle.drive_speed * -0.5

        wait(500)

        straight_start_time = time.time()
        while time.time() - straight_start_time <= 1.5:
            self.vehicle._drive_base.drive(drive_speed=reverse_drive_speed, turn_rate=0)

        turn_start_time = time.time()
        while time.time() - turn_start_time <= 1.7:
            self.vehicle._drive_base.drive(drive_speed=reverse_drive_speed, turn_rate=-50)

        wait(500)

        straight_start_time = time.time()
        while time.time() - straight_start_time <= 1.5:
            self.vehicle._drive_base.drive(drive_speed=reverse_drive_speed, turn_rate=0)

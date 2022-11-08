import time
from abc import ABC, abstractmethod


class State(ABC):
    @abstractmethod
    def drive(self): pass

    @abstractmethod
    def change_lane(self): pass

    @abstractmethod
    def pause(self): pass

    @abstractmethod
    def slow_down(self): pass


class DrivingState(State):
    def __init__(self, machine):
        self.machine = machine

    def drive(self):
        pass

    def change_lane(self):
        pass

    def pause(self):
        pass

    def slow_down(self):
        pass


class ObstacleDetectedState(State):
    def __init__(self, machine):
        self.machine = machine

    def drive(self):
        pass

    def change_lane(self):
        # logic of change lane
        self.machine.set_state(self.machine.get_driving_state())

    def pause(self):
        pass

    def slow_down(self):
        pass


class BreakBlockDetectedState(State):
    def __init__(self, machine):
        self.machine = machine

    def drive(self):
        pass

    def change_lane(self):
        pass

    def pause(self):
        pass

    def slow_down(self):
        pass


class SchoolZoneDetectedState(State):
    def __init__(self, machine):
        self.machine = machine

    def drive(self):
        pass

    def change_lane(self):
        pass

    def pause(self):
        pass

    def slow_down(self):
        pass


class LabFinishedState(State):
    def __init__(self, machine):
        self.machine = machine

    def drive(self):
        pass

    def change_lane(self):
        pass

    def pause(self):
        pass

    def slow_down(self):
        pass

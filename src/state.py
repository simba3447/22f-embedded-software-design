from abc import ABC, abstractmethod

from src.machine import Machine


class State(ABC):
    def __init__(self, machine: Machine):
        self.machine = machine

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
        super(DrivingState, self).__init__(machine)

    def drive(self):
        while True:
            if self.machine.detect_pause_block():
                self.machine.set_state(self.machine.get_pause_block_detected_state())

            if self.machine.detect_school_zone_block():
                self.machine.set_state(self.machine.get_school_zone_block_detected_state())

            self.machine.drive()

    def change_lane(self):
        pass

    def pause(self):
        pass

    def slow_down(self):
        pass


class ObstacleDetectedState(State):
    def __init__(self, machine):
        super(ObstacleDetectedState, self).__init__(machine)

    def drive(self):
        pass

    def change_lane(self):
        # logic of change lane
        self.machine.set_state(self.machine.get_driving_state())

    def pause(self):
        pass

    def slow_down(self):
        pass


class PauseBlockDetectedState(State):
    def __init__(self, machine):
        super(PauseBlockDetectedState, self).__init__(machine)

    def drive(self):
        pass

    def change_lane(self):
        pass

    def pause(self):
        self.machine.pause()
        self.machine.set_state(self.machine.get_driving_state())

    def slow_down(self):
        pass


class SchoolZoneDetectedState(State):
    def __init__(self, machine):
        super(SchoolZoneDetectedState, self).__init__(machine)

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
        super(LabFinishedState, self).__init__(machine)

    def drive(self):
        pass

    def change_lane(self):
        pass

    def pause(self):
        pass

    def slow_down(self):
        pass

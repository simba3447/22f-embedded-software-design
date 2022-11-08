from src.state import DrivingState, ObstacleDetectedState, BreakBlockDetectedState, SchoolZoneDetectedState, \
    LabFinishedState, State


class Machine:
    def __init__(self):
        self.driving_state = DrivingState(self)
        self.obstacle_detected_state = ObstacleDetectedState(self)
        self.break_block_detected_state = BreakBlockDetectedState(self)
        self.school_zone_detected_state = SchoolZoneDetectedState(self)
        self.lab_finished_state = LabFinishedState(self)

        self.state = self.driving_state

    def drive(self):
        self.driving_state.drive()

    def set_state(self, state: State):
        self.state = state

    def get_driving_state(self):
        return self.driving_state

from pybricks.ev3devices import ColorSensor, UltrasonicSensor
from pybricks.hubs import EV3Brick
from pybricks.parameters import Port
from pybricks.pupdevices import Motor
from pybricks.robotics import DriveBase
from pybricks.tools import wait

from src.enums import Lane
from src.state import DrivingState, ObstacleDetectedState, PauseBlockDetectedState, SchoolZoneDetectedState, \
    LabFinishedState, State


class Machine:
    def __init__(self):
        self.driving_state = DrivingState(self)
        self.obstacle_detected_state = ObstacleDetectedState(self)
        self.pause_block_detected_state = PauseBlockDetectedState(self)
        self.school_zone_detected_state = SchoolZoneDetectedState(self)
        self.lab_finished_state = LabFinishedState(self)

        self.state = self.driving_state
        self.current_lane = Lane.FIRST_LANE

        self.left_color_sensor = ColorSensor(Port.S1)
        self.right_color_sensor = ColorSensor(Port.S4)
        self.distance_sensor = UltrasonicSensor(Port.S3)
        self.park_sensor = UltrasonicSensor(Port.S2)

        self.motor_left = Motor(Port.B)
        self.motor_right = Motor(Port.C)

        self.ev3_brick = EV3Brick()
        self.drive_base = DriveBase(self.motor_left, self.motor_right, wheel_diameter=55.5, axle_track=104)

    def drive(self):
        self.driving_state.drive()

    def set_state(self, state: State):
        self.state = state

    def get_driving_state(self):
        return self.driving_state

    def pause(self):
        pause_duration_milliseconds = 3000

        self._beep()
        self._stop()
        wait(pause_duration_milliseconds)

    def detect_blue_color(self):
        return any([self.left_color_sensor.rgb(), self.right_color_sensor.color()])

    def _beep(self):
        self.ev3_brick.speaker.beep()

    def _stop(self):
        self.drive_base.stop()

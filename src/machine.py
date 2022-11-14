from pybricks.ev3devices import Motor, ColorSensor, UltrasonicSensor
from pybricks.hubs import EV3Brick
from pybricks.parameters import Port
from pybricks.robotics import DriveBase
from pybricks.tools import wait

from src.enums import Lane
from src.objectdetector import RedColorDetector, BlueColorDetector, YellowColorDetector
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

        self.blue_color_detector = BlueColorDetector(queue_len=2, decision_criteria=1)
        self.yellow_color_detector = YellowColorDetector(queue_len=2, decision_criteria=1)
        self.red_color_detector = RedColorDetector(queue_len=2, decision_criteria=1)

    def drive(self):
        self.driving_state.drive()

    def set_state(self, state: State):
        self.state = state

    def get_driving_state(self):
        return self.driving_state

    def get_pause_block_detected_state(self):
        return self.pause_block_detected_state

    def get_school_zone_block_detected_state(self):
        return self.school_zone_detected_state

    def get_lab_finished_state(self):
        return self.lab_finished_state

    def pause(self):
        pause_duration_milliseconds = 3000

        self._beep()
        self._stop()
        wait(pause_duration_milliseconds)

    def detect_pause_block(self):
        return self.blue_color_detector.color_detected(self.left_color_sensor.rgb(), self.right_color_sensor.rgb())

    def detect_school_zone_block(self):
        return self.yellow_color_detector.color_detected(self.left_color_sensor.rgb(), self.right_color_sensor.rgb())

    def detect_lab_finish_line_block(self):
        return self.red_color_detector.color_detected(self.left_color_sensor.rgb(), self.right_color_sensor.rgb())

    def _beep(self):
        self.ev3_brick.speaker.beep()

    def _stop(self):
        self.drive_base.stop()

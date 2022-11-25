import time
from abc import abstractmethod


class ObjectDetector:
    def __init__(self, queue_len: int, decision_criteria: int):
        if decision_criteria > queue_len:
            raise Exception("Error: Value of decision criteria could not be more than the length of queue")

        self.queue_len = queue_len
        self.decision_criteria = decision_criteria

        self.detection_result_queue = []

    def detect_object(self):
        return len(self.detection_result_queue) == self.queue_len and sum(
            self.detection_result_queue) >= self.decision_criteria

    def add_detection_result(self, result: bool):
        if len(self.detection_result_queue) == self.queue_len:
            self.detection_result_queue.pop(0)

        self.detection_result_queue.append(result)

    def reset_detection_result(self):
        self.detection_result_queue = []


class ColorDetector(ObjectDetector):
    class RGB:
        def __init__(self, red: int, green: int, blue: int):
            self.red = red
            self.green = green
            self.blue = blue

        def __str__(self):
            return "({}) RED: {:>3d}, GREEN: {:>3d}, BLUE: {:>3d}".format(self.__class__.__name__, self.red, self.green, self.blue)

    RGB_LOWER_BOUND = 30
    RGB_UPPER_BOUND = 20

    SEPARATE_DETECTION_INTERVAL_SECONDS = 2

    def __init__(self, queue_len: int, decision_criteria: int, color_sensor_list: list):
        super(ColorDetector, self).__init__(queue_len=queue_len, decision_criteria=decision_criteria)

        self.color_sensor_list = color_sensor_list
        self.last_detection_time = 0

    def color_detected(self):
        self.add_detection_result(any(
            self._color_detected(color_sensor.rgb()) for color_sensor in self.color_sensor_list,
        ))
        if self.detect_object() and self._exceed_last_detection_interval():
            self.last_detection_time = time.time()
            return True
        else:
            return False

    def _exceed_last_detection_interval(self):
        current_time = time.time()

        return current_time - self.last_detection_time >= self.SEPARATE_DETECTION_INTERVAL_SECONDS

    def _color_detected(self, rgb: tuple[int, int, int]):
        red, green, blue = rgb

        return self.color_decision_criteria(self.RGB(red, green, blue))

    @abstractmethod
    def color_decision_criteria(self, rgb: RGB):
        raise Exception("Decision Criteria Not Defined")


class RedColorDetector(ColorDetector):
    def color_decision_criteria(self, rgb: ColorDetector.RGB):
        return rgb.red > self.RGB_LOWER_BOUND and rgb.green < self.RGB_UPPER_BOUND and rgb.blue < self.RGB_UPPER_BOUND


class BlueColorDetector(ColorDetector):
    def color_decision_criteria(self, rgb: ColorDetector.RGB):
        return rgb.red > self.RGB_LOWER_BOUND and rgb.green < self.RGB_UPPER_BOUND and rgb.blue < self.RGB_UPPER_BOUND


class YellowColorDetector(ColorDetector):
    def color_decision_criteria(self, rgb: ColorDetector.RGB):
        return rgb.red > self.RGB_LOWER_BOUND and rgb.green < self.RGB_UPPER_BOUND and rgb.blue < self.RGB_UPPER_BOUND

class ObstacleDetector(ObjectDetector):
    OBSTACLE_DETECT_DISTANCE_MILLIMETER = 250
    SEPARATE_DETECTION_INTERVAL_SECONDS = 2

    def __init__(self, queue_len: int, decision_criteria: int, ultrasonic_sensor):
        super(ObstacleDetector, self).__init__(queue_len=queue_len, decision_criteria=decision_criteria)

        self.ultrasonic_sensor = ultrasonic_sensor
        self.last_detection_time = 0

    def obstacle_detected(self):
        self.add_detection_result(int(self.ultrasonic_sensor.distance()) < self.OBSTACLE_DETECT_DISTANCE_MILLIMETER)

        if self.detect_object() and self._exceed_last_detection_interval():
            self.last_detection_time = time.time()
            return True
        else:
            return False

    def _exceed_last_detection_interval(self):
        current_time = time.time()

        return current_time - self.last_detection_time >= self.SEPARATE_DETECTION_INTERVAL_SECONDS

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

    RGB_LOWER_BOUND = 30
    RGB_UPPER_BOUND = 20

    SEPARATE_DETECTION_INTERVAL_SECONDS = 2

    def __init__(self, queue_len: int, decision_criteria: int):
        super(ColorDetector, self).__init__(queue_len=queue_len, decision_criteria=decision_criteria)

        self.last_detection_time = 0

    def color_detected(self, rgb_left: tuple[int, int, int], rgb_right: tuple[int, int, int]):
        self.add_detection_result(any([
            self._color_detected(rgb_left),
            self._color_detected(rgb_right),
        ]))

        return self.detect_object() and self._exceed_last_detection_interval()

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
        return rgb.red > ColorDetector.RGB_LOWER_BOUND and rgb.green < ColorDetector.RGB_UPPER_BOUND and rgb.blue < ColorDetector.RGB_UPPER_BOUND


class BlueColorDetector(ColorDetector):
    def color_decision_criteria(self, rgb: ColorDetector.RGB):
        return rgb.red > ColorDetector.RGB_LOWER_BOUND and rgb.green < ColorDetector.RGB_UPPER_BOUND and rgb.blue < ColorDetector.RGB_UPPER_BOUND


class YellowColorDetector(ColorDetector):
    def color_decision_criteria(self, rgb: ColorDetector.RGB):
        return rgb.red > ColorDetector.RGB_LOWER_BOUND and rgb.green < ColorDetector.RGB_UPPER_BOUND and rgb.blue < ColorDetector.RGB_UPPER_BOUND

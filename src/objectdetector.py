import time
from abc import abstractmethod

from pybricks.parameters import Color


class ObjectDetector:
    UNIQUE_OBJECT_DETECTION_INTERVAL_SECONDS = 2

    def __init__(self, queue_len: int, threshold: int, enabled: bool = True):
        if threshold > queue_len:
            raise Exception("Error: Value of decision criteria could not be more than the length of queue")

        self.queue_len = queue_len
        self.threshold = threshold

        self.detection_result_queue = []
        self.last_detection_time = 0

        self._enabled = enabled

    def detected(self):
        if not self._enabled:
            return False

        self._add_detection_result(self.decision_criteria())
        if self._exceed_detection_threshold() and self._exceed_last_detection_interval():
            self._update_last_detection_time()
            self._reset_detection_result()
            return True
        else:
            return False

    @abstractmethod
    def decision_criteria(self):
        pass

    def enable(self):
        self._enabled = True

    def disable(self):
        self._enabled = False

    def _add_detection_result(self, result: bool):
        if len(self.detection_result_queue) == self.queue_len:
            self.detection_result_queue.pop(0)

        self.detection_result_queue.append(result)

    def _exceed_detection_threshold(self):
        return len(self.detection_result_queue) == self.queue_len and sum(
            self.detection_result_queue) >= self.threshold

    def _exceed_last_detection_interval(self):
        current_time = time.time()

        return current_time - self.last_detection_time >= self.UNIQUE_OBJECT_DETECTION_INTERVAL_SECONDS

    def _update_last_detection_time(self):
        self.last_detection_time = time.time()

    def _reset_detection_result(self):
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

    def __init__(self, queue_len: int, threshold: int, color_sensor_list: list, enabled: bool = True):
        super(ColorDetector, self).__init__(queue_len=queue_len, threshold=threshold, enabled=enabled)

        self.color_sensor_list = color_sensor_list

    def decision_criteria(self):
        return any(
            self._color_detected(color_sensor.rgb()) for color_sensor in self.color_sensor_list,
        )

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
        return rgb.red < 20 and rgb.green < 30 and rgb.blue > 30


class YellowColorDetector(ColorDetector):
    def color_decision_criteria(self, rgb: ColorDetector.RGB):
        return rgb.red > 30 and rgb.green > 30 and rgb.blue < 20


class ObstacleDetector(ObjectDetector):
    OBSTACLE_DETECT_DISTANCE_MILLIMETER = 250

    def __init__(self, queue_len: int, threshold: int, ultrasonic_sensor, enabled=True):
        super(ObstacleDetector, self).__init__(queue_len=queue_len, threshold=threshold, enabled=enabled)

        self.ultrasonic_sensor = ultrasonic_sensor

    def decision_criteria(self):
        return int(self.ultrasonic_sensor.distance()) < self.OBSTACLE_DETECT_DISTANCE_MILLIMETER


class ParkingLotDetector(ObstacleDetector):
    OBSTACLE_DETECT_DISTANCE_MILLIMETER = 100


class SimpleColorDetector(ObjectDetector):
    decision_color = None

    def __init__(self, queue_len: int, threshold: int, color_sensor_list: list, enabled: bool = True):
        super(SimpleColorDetector, self).__init__(queue_len=queue_len, threshold=threshold, enabled=enabled)

        self.color_sensor_list = color_sensor_list
        self.last_detection_time = 0

    def decision_criteria(self):
        return any(
            self._color_detected(color_sensor) for color_sensor in self.color_sensor_list,
        )

    def _color_detected(self, color_sensor):
        return color_sensor.color() == self.decision_color


class SimpleRedColorDetector(SimpleColorDetector):
    decision_color = Color.RED


class SimpleBlueColorDetector(SimpleColorDetector):
    decision_color = Color.BLUE


class SimpleYellowColorDetector(SimpleColorDetector):
    decision_color = Color.YELLOW

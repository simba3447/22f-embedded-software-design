import time

from pybricks.tools import DataLog, StopWatch


class EventLogger:
    INFO = 'INFO'
    DEBUG = 'DEBUG'
    WARNING = 'WARNING'
    ERROR = 'ERROR'

    def __init__(self):
        self._logger = DataLog('time', 'elapsed_time', 'level', 'message', name='event_log')
        self._stop_watch = StopWatch()

    def _log(self, level, message):
        now_time = time.localtime(time.time())
        now_time_formatted = time.strftime('%Y-%m-%d %I:%M:%S %p', now_time)

        elapsed_time = self._stop_watch.time() / 1000
        self._logger.log(now_time_formatted, elapsed_time, level, message)
        print('{}, {:8.4f}, {}, {}'.format(now_time_formatted, elapsed_time, level, message))

    def info(self, message):
        self._log(self.INFO, message)

    def debug(self, message):
        self._log(self.DEBUG, message)

    def warning(self, message):
        self._log(self.WARNING, message)

    def error(self, message):
        self._log(self.ERROR, message)


class ColorSensorLogger:
    INFO = 'INFO'
    DEBUG = 'DEBUG'
    WARNING = 'WARNING'
    ERROR = 'ERROR'

    def __init__(self):
        self._logger = DataLog('elapsed_time', 'left_sensor_color', 'left_sensor_r', 'left_sensor_g', 'left_sensor_b', 'right_sensor_color', 'right_sensor_r','right_sensor_g','right_sensor_b', name="color_sensor_log")
        self._stop_watch = StopWatch()

    def log(self, left_sensor_color, left_sensor_rgb, right_sensor_color, right_sensor_rgb):
        self._logger.log(self._stop_watch.time(), left_sensor_color, left_sensor_rgb[0], left_sensor_rgb[1], left_sensor_rgb[2], right_sensor_color, right_sensor_rgb[0], right_sensor_rgb[1], right_sensor_rgb[2])

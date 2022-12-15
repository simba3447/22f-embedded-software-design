import time

from pybricks.tools import DataLog, StopWatch


class EventLogger:
    INFO = 'INFO'
    WARNING = 'WARNING'
    ERROR = 'ERROR'

    def __init__(self):
        self._logger = DataLog('time', 'elapsed_time', 'level', 'message')
        self._stop_watch = StopWatch()

    def _log(self, level, message):
        now_time = time.localtime(time.time())
        now_time_formatted = time.strftime('%Y-%m-%d %I:%M:%S %p', now_time)

        elapsed_time = self._stop_watch.time() / 1000
        self._logger.log(now_time_formatted, elapsed_time, level, message)
        print('{}, {:8.4f}, {}, {}'.format(now_time_formatted, elapsed_time, level, message))

    def info(self, message):
        self._log(self.INFO, message)

    def warning(self, message):
        self._log(self.WARNING, message)

    def error(self, message):
        self._log(self.ERROR, message)

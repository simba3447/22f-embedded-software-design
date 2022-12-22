import time


class WrongMessageFormat(ValueError):
    pass


class PlatooningMessage:
    STOP_SIGN = 'blue'
    OBSTACLE_DETECTED_SIGN = 'ob'
    SCHOOL_ZONE_SIGN = 'yellow'
    LAB_END_SIGN = 'red'
    PARKING_LOT_DETECTED = 'parking-lot-detected'
    PARKING_ENDED = 'parking-ended'

    def __init__(self, code: str, timestamp: int):
        self.code = code
        self.timestamp = timestamp

    def __str__(self):
        return "{}:{}".format(self.code, self.timestamp)

    @classmethod
    def from_data(cls, data: str):
        try:
            code, timestamp = data.split(":")
            timestamp = int(timestamp)
        except ValueError:
            raise WrongMessageFormat

        return cls(code=code, timestamp=timestamp)

    @classmethod
    def from_code(cls, code: str):
        timestamp = int(time.time())

        return cls(code=code, timestamp=timestamp)


class PlatooningMessageRegistry:
    _data = {}

    def insert(self, message: PlatooningMessage):
        if message.code not in self._data:
            self._data[message.code] = [message.timestamp]
        else:
            self._data[message.code].append(message.timestamp)

    def find(self, message):
        return message.code in self._data and message.timestamp in self._data[message.code]

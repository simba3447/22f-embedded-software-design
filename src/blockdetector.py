class BlockDetector:
    def __init__(self, queue_len: int, decision_criteria: int):
        if decision_criteria > queue_len:
            raise Exception("Error: Value of decision criteria could not be more than the length of queue")

        self.queue_len = queue_len
        self.decision_criteria = decision_criteria

        self.detection_result_queue = []

    def detect_block(self):
        return len(self.detection_result_queue) == self.queue_len and sum(
            self.detection_result_queue) >= self.decision_criteria

    def add_detection_result(self, result: bool):
        if len(self.detection_result_queue) == self.queue_len:
            self.detection_result_queue.pop(0)

        self.detection_result_queue.append(result)

    def reset_detection_result(self):
        self.detection_result_queue = []

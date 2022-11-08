from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, ColorSensor, UltrasonicSensor)
from pybricks.parameters import Port
from pybricks.tools import wait
from pybricks.robotics import DriveBase

import time

##########################
### Start of Constants ###

DRIVE_SPEED = 100
PROPORTIONAL_GAIN = 0.8
BREAK_BLOCK_DETECT_COUNT_THRESHOLD = 3
OBSTARCLE_BLOCK_DETECT_COUNT_THRESHOLD = 3
PARK_BLOCK_DETECT_COUNT_THRESHOLD = 3

RGB_LOWER_BOUND = 30
RGB_UPPER_BOUND = 20

#### End of Constants ####
##########################

# class Lane(enum):
FIRST_LANE = 0
SECOND_LANE = 1

class BlockDetector:
    def __init__(self, queue_len:int, decision_criteria:int):
        if decision_criteria > queue_len:
            raise Exception("Error: Value of decision criteria could not be more than the length of queue")

        self.queue_len = queue_len
        self.decision_criteria = decision_criteria

        self.detection_result_queue = []

    def detect_block(self):
        return len(self.detection_result_queue) == self.queue_len and sum(self.detection_result_queue) >= self.decision_criteria

    def add_detection_result(self, result:bool):
        if len(self.detection_result_queue) == self.queue_len:
            self.detection_result_queue.pop(0)
            
        self.detection_result_queue.append(result)
    
    def reset_detection_result(self):
        self.detection_reesult_queue = []

class Vehicle:
    def __init__(self, port_motor_l, port_motor_r, port_sensor_l, port_sensor_r, port_sensor_distance, port_sensor_park):
        self.motor_l = Vehicle.initialize_motor(port_motor_l)
        self.motor_r = Vehicle.initialize_motor(port_motor_r)

        self.ev3 = EV3Brick()
        self.drive_base = DriveBase(self.motor_l, self.motor_r, wheel_diameter=55.5, axle_track=104)
        
        self.sensor_l = Vehicle.initialize_color_sensor(port_sensor_l)
        self.sensor_r = Vehicle.initialize_color_sensor(port_sensor_r)

        self.distance_sensor = UltrasonicSensor(port_sensor_distance)
        self.park_sensor = UltrasonicSensor(port_sensor_park)

        self.last_break_datetime = time.time()
        self.last_obstarcle_datetime = time.time()
        self.lab_end_block_datetime = time.time()
        self.park_block_datetime = time.time()

        self.break_block_count = 0
        self.terminate_block_count = 0
        self.obstarcle_block_count = 0
        self.park_block_count = 0

        self.obstarcle_block_detector = BlockDetector(queue_len = 6, decision_criteria = 3)
        # TODO: Improve block detection method to use statistics
        self.red_block_detector = BlockDetector(queue_len = 3, decision_criteria = 1)
        self.yellow_block_detector = BlockDetector(queue_len = 3, decision_criteria = 1)
        self.blue_block_detector = BlockDetector(queue_len = 3, decision_criteria = 1)

        self.current_lane = FIRST_LANE

        self.lab_finished = False
        self.detect_first_parking_sign = False

    
    @classmethod
    def initialize_motor(cls, port_motor):
        return Motor(port_motor)

    @classmethod
    def initialize_color_sensor(cls, port_sensor):
        return ColorSensor(port_sensor)

    @classmethod
    def detect_blue_color(cls, rgb_data):
        red, green, blue = rgb_data 

        return red < RGB_UPPER_BOUND and green < RGB_UPPER_BOUND and blue > RGB_LOWER_BOUND

    @classmethod
    def detect_red_color(cls, rgb_data):
        red, green, blue = rgb_data

        return red > RGB_LOWER_BOUND and green < RGB_UPPER_BOUND and blue < RGB_UPPER_BOUND

    @classmethod
    def detect_yellow_color(cls, rgb_data):
        red, green, blue = rgb_data

        return red > RGB_LOWER_BOUND and green > RGB_LOWER_BOUND and blue < RGB_UPPER_BOUND

    def drive(self):
        self.ev3.speaker.beep()

        while True:
            turn_rate = self._get_turn_rate()

            self.pause_vehicle()
            self.change_lane()
            
            self.slow_down_vehicle()

            if Vehicle.detect_red_color(self.sensor_l.rgb()) or Vehicle.detect_red_color(self.sensor_r.rgb()):
                print("!!!!!!!!! RED COLOR DETECTED !!!!!!!!!")

                self.red_block_detector.add_detection_result(True)

                if not self.red_block_detector.detect_block():
                    pass
                if time.time() - self.lab_end_block_datetime < 3:
                    pass
                
                self.ev3.speaker.beep()
                self.lab_finished = True
                self.lab_end_block_datetime = time.time()
                
            else:
                self.red_block_detector.add_detection_result(False)

            if self.lab_finished and self.detect_parking():
                if time.time() - self.park_block_datetime < 2:
                    continue
                else:
                    if not self.detect_first_parking_sign:
                        self.detect_first_parking_sign = True
                        self.park_block_datetime = time.time()
                        continue
                    else:
                        self.ev3.speaker.beep()
                        self.start_parking()
                        self.drive_base.stop()
                        self.ev3.speaker.beep()
                        return
            
            self.drive_base.drive(DRIVE_SPEED, turn_rate)
    
    def _get_turn_rate(self):
        deviation = self.sensor_l.reflection() - self.sensor_r.reflection()
        return PROPORTIONAL_GAIN * deviation

    def detect_break_block(reseult:bool):
        QUEUE_LEN = 5
        DICISION_CRITERIA = 5

        current_detection_result_queue = []
    

    def detect_obstarcle(reseult:bool):
        QUEUE_LEN = 5
        DICISION_CRITERIA = 5

        current_detection_result_queue = []

        
    def pause_vehicle(self):
        if (Vehicle.detect_blue_color(self.sensor_l.rgb()) or Vehicle.detect_blue_color(self.sensor_r.rgb())):
            self.blue_block_detector.add_detection_result(True)

            print("!!!!!!!!! BLUE COLOR DETECTED !!!!!!!!!")
            if time.time() - self.last_break_datetime < 2:
                return
            
            if not self.blue_block_detector.detect_block():
                return
            # if self.break_block_count < BREAK_BLOCK_DETECT_COUNT_THRESHOLD:
            #     self.break_block_count += 1
            #     return
            self.ev3.speaker.beep()
            self.drive_base.stop()
            wait(3000)

            self.last_break_datetime = time.time()
            self.break_block_count = 0
            self.blue_block_detector.reset_detection_result()
        else:
            self.blue_block_detector.add_detection_result(False)

    def slow_down_vehicle(self):
        SLOW_DRIVE_SPEED = DRIVE_SPEED / 2
        SLOW_DRIVE_DURATION = 2.5

        if (Vehicle.detect_yellow_color(self.sensor_l.rgb()) or Vehicle.detect_yellow_color(self.sensor_r.rgb())):
            print("!!!!!!!!! YELLOW COLOR DETECTED !!!!!!!!!")

            self.yellow_block_detector.add_detection_result(True)
            
            if not self.yellow_block_detector.detect_block():
                return
            
            if time.time() - self.last_break_datetime < 2:
                return

            self.ev3.speaker.beep()
            slow_down_start_time = time.time()

            while True:
                if time.time() - slow_down_start_time > SLOW_DRIVE_DURATION:
                    break

                turn_rate = self._get_turn_rate()

                self.drive_base.drive(SLOW_DRIVE_SPEED, turn_rate)
            self.last_break_datetime = time.time()
        else:
            self.yellow_block_detector.add_detection_result(False)


    def change_lane(self):
        OBSTARCLE_DETECT_DISTANCE = 250

        if int(self.distance_sensor.distance()) < OBSTARCLE_DETECT_DISTANCE:
            # TODO: Improve previous obstarcle log comparison
            # if time.time() - self.last_obstarcle_datetime < 4:
            #     self.obstarcle_block_detector.add_detection_result(False)
            #     return

            if not self.obstarcle_block_detector.detect_block():
                self.obstarcle_block_detector.add_detection_result(True)
                return

            self._change_lane()
        else:
            self.obstarcle_block_detector.add_detection_result(False)


    def _change_lane(self):
        TURN_DURATION_SECONDS = 0.8
        RETURN_DURATION_SECONDS = 0.8
        STRAIGHT_DRIVE_DURATION_SECONDS = 0
        proportional_gain = 0.13

        # Decide turn direction
        if self.current_lane == SECOND_LANE:
            proportional_gain *= -1

        turn_rate = self._get_turn_rate()
        
        turn_start_time = time.time()
        while True:
            if time.time() - turn_start_time > TURN_DURATION_SECONDS:
                break
            turn_rate += proportional_gain
            self.drive_base.drive(DRIVE_SPEED, turn_rate)
        
        turn_start_time = time.time()
        while True:
            if time.time() - turn_start_time > RETURN_DURATION_SECONDS * 2:
                break

            turn_rate -= proportional_gain

            self.drive_base.drive(DRIVE_SPEED, turn_rate)
        
        weight = 0
        weight_increment_amount = 0.03
        turn_start_time = time.time()
        while True:
            if time.time() - turn_start_time > TURN_DURATION_SECONDS:
                break
            turn_rate += proportional_gain

            temp_turn_rate = self._get_turn_rate()

            if weight + weight_increment_amount < 1:
                weight += weight_increment_amount

            total_turn_rate = turn_rate * (1 - weight) + temp_turn_rate * weight
            # print("weight: " + "{:10.4f}".format(weight) + ", total_turn_rate: " + "{:10.4f}".format(total_turn_rate))
            
            self.drive_base.drive(DRIVE_SPEED, total_turn_rate)
        # print("!!!!!!!!!!!! CHANGING LANE DONE")
        self.last_obstarcle_datetime = time.time()
        self.obstarcle_block_detector.reset_detection_result()

        # Change current lane state
        if self.current_lane == FIRST_LANE:
            self.current_lane = SECOND_LANE
        else:
            self.current_lane = FIRST_LANE

    def detect_parking(self):
        # print(self.park_sensor.distance())
        PARK_DETECT_DISTANCE_MILLIMETER = 200
        if int(self.park_sensor.distance()) < PARK_DETECT_DISTANCE_MILLIMETER:
            if self.park_block_count < OBSTARCLE_BLOCK_DETECT_COUNT_THRESHOLD:
                self.park_block_count += 1
                return False
            return True
        return False

    def start_parking(self):
        REVERSE_DRIVE_SPEED = DRIVE_SPEED * -0.5
        TURN_DURATION_SECONDS = 1
        RETURN_DURATION_SECONDS = 1
        STRAIGHT_DRIVE_DURATION_SECONDS = 1
        
        proportional_gain = -0.2
        
        wait(500)
        
        turn_rate = 0
        
        straight_start_time = time.time()
        while True:
            if time.time() - straight_start_time > 1.5:
                break
            self.drive_base.drive(REVERSE_DRIVE_SPEED, turn_rate)

        turn_start_time = time.time()

        while True:
            if time.time() - turn_start_time > 1.5:
                break
            turn_rate = -50

            self.drive_base.drive(REVERSE_DRIVE_SPEED, turn_rate)
        
        wait(500)

        turn_rate = 0
        straight_start_time = time.time()     
        while True:
            if time.time() - straight_start_time > 0.75:
                break
            self.drive_base.drive(REVERSE_DRIVE_SPEED, turn_rate)

        wait(1000)

        turn_start_time = time.time()
        while True:
            if time.time() - turn_start_time > 2:
                break
            turn_rate = 50

            self.drive_base.drive(REVERSE_DRIVE_SPEED, turn_rate)

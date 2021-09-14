"""Simple MicroPython script which demonstrates uPyBot controlled using the Micro Python Board (pyboard)"""
# 25-Dec-2019 Initial version of the script created
# 16-Jan-2020 Simple update robot class renamed to uPyBot
# 05-Sep-2021 Changed motor methods (added delay between movements and removed fats stop) to avoid high current load which reset uPyBot
# 11-Sep-2021 Added methods to utilize Husky Lena AI camera for my own face recognition and greeting by turn_for_hello() method
# 12-Sep-2021 Added blue LED to signal when uPyBot is ready to recognize my face

import pyb
from pyb import Pin, Timer
from pyb import ExtInt, LED
from huskylensPythonLibrary import HuskyLensLibrary
import micropython
from micropython import const
import utime as time
import urandom as random

micropython.alloc_emergency_exception_buf(100) #recommended by MicroPython doc to allow produce error msg in interrupts

class Beeper:
    """ This class controls simple buzzer"""
    BEEP_DEFAULT_DURATION_MS = const(200)
    def __init__(self, ctrl_pin):
        self._ctrl_pin = ctrl_pin #pyboard pin used to control beeper
        self._ctrl_pin.low()

    def beep(self,duration):
        self._ctrl_pin.high() #turn beeper on
        pyb.delay(duration)   #wait specified time
        self._ctrl_pin.low()  #turn beeper off

class Encoder:
    """This class control WSR-08834 encoder. One of the pyboard pin is used to count pulses (via an interrupt)"""
    def __init__(self, robot, int_pin):
        self._robot = robot  #reference to robot an encoder belongs into (is used to stop its motors) 
        self._int_pin = int_pin #pyboard pin used to count encoder pulses
        self._cnt_pulses = 0   #counts pulses from encoder
        self._stop_pulses = 0  #keeps destination number of pulses to stop motor when reached 
        self._last_pulses = 0  #number of counted pulses during last check
        self._ext_int = ExtInt(int_pin, ExtInt.IRQ_RISING,  Pin.PULL_NONE, self.__int_handler) #setup pin to trigger interrupt

    def __int_handler(self,line): #encoder interrupt handler
        self._cnt_pulses += 1
        if self._cnt_pulses >= self._stop_pulses: #when destination number of pulses reached stop robot's motor
            self._ext_int.disable()
            self._robot.stop()
            
    def set_stop_pulses(self, stop_pulses): #setup destination number of pulses for the move
        self._cnt_pulses = 0
        self._last_pulses = 0
        self._stop_pulses = stop_pulses
        self._ext_int.enable()

    def get_stop_pulses(self):
        return self._stop_pulses

    def get_current_pulses(self):
        return self._cnt_pulses

    def force_stop_cnt(self):
        # change counters so movement end is forced to be reported by is_stopped method
        self._cnt_pulses = self._stop_pulses

    def reset(self):
        self._cnt_pulses = 0
        self._stop_pulses = 0
        self._last_pulses = 0

    def is_stopped(self): #check if movement ended i.e. if number of desired pulses reached
        if self._cnt_pulses >= self._stop_pulses:
            return True
        else:
            return False

    def is_moving(self): #check if uPyBot moving
        return not self.is_stopped()

    def is_changing(self): #check if number of pulses changed since last check
        changing = False
        if self._cnt_pulses != self._last_pulses:
            changing = True
        self._last_pulses = self._cnt_pulses
        return changing
                
class Motor:
    """This class control robot's motor through L298 driver"""
    def __init__(self,pinA, pinB, pwm_ch):
        self._pinA = pinA #setup pins which control motor move direction
        self._pinB = pinB
        self._pwm_ch = pwm_ch #setup pwm channel used to control motor speed

    def set_forward(self):
        self._pinA.high()
        self._pinB.low()

    def set_backward(self):
        self._pinA.low()
        self._pinB.high()

    def stop(self):
        self.set_speed(0) #setup PWM output to low this should be enough to stop
    #    self._pinA.low() removed to not generate high current load which caused AI camera reset
    #    self._pinB.low()

    def set_speed(self,speed): #setup motor speed (which is pwm pulse width in precentage 0-100)
        self._pwm_ch.pulse_width_percent(speed)

class UltraSonicSensor:
    """This class uses US HC-SR04 to make obstacle detection"""
    def __init__(self,pin_trig,pin_echo,us_tmr):
        self._pin_trigger = pin_trig #pin used to trigger sensor
        self._pin_trigger.low()
        self._pin_echo = pin_echo #pin which detects sensor echo response
        self._us_tmr = us_tmr
    
    def distance_to_obstacle_in_cm(self): #trigger us sensor to measure distance to the closest obstacle
        start = 0
        stop = 0
        self._pin_trigger.low()
        self._us_tmr.counter(0)

        # Send a 10us pulse - this triggers measurement sequence
        self._pin_trigger.high()
        pyb.udelay(10)
        self._pin_trigger.low()
    
        # Wait 'till the pulse starts.
        while self._pin_echo.value() == 0:
            start = self._us_tmr.counter()

        # Wait 'till the pulse is gone.
        while self._pin_echo.value() == 1:
            stop = self._us_tmr.counter()
        
        #calculate a distance in cm
        dist_in_cm = ((stop - start) / 2) / 29 #convert response pulse duration into distance
        return dist_in_cm
    
class uPyBot:
    """This class represent uPyBot robot"""

    #define various useful constants used by uPyBot programs
    LOW_SPEED = const(85)
    MID_SPEED = const(95)
    HIGH_SPEED = const(100)

    TURN_45 = const(3)
    TURN_90 = const(6)
    TURN_180 = const(17)
    TURN_360 = const(34)

    WHEEL_TURN_90 = const(17)
    WHEEL_TURN_180 = const(35)
    WHEEL_TURN_360 = const(70)

    STEP_BACK_DISTANCE = const(7)
    NEXT_MOVE_TIMEOUT_MS = const(200) #wait between subsequent movements to not overload system by motor rapidly changing

    LOW_DISTANCE = const(10)

    STALL_CHECK_TIME_MS = const (100)

    MOVE_OK = const(0)
    MOVE_OBSTACLE = const(1)
    MOVE_STALL = const(2)
    MOVE_FACE_RECOGNIZED = const(3)

    MOVE_MIN = const(10)
    MOVE_STEP = const(10)
    MOVE_MAX = const(100)

    SYMMETRIC_TURN = const(1)
    ASYMMETRIC_TURN = const(2)
    LEFT_TURN = const(1)
    RIGHT_TURN = const(2)
    HELLO_TURN_CNT = const(5)

    FACE_ID_TO_RECOGNIZE = const(1) #HuskyLens ID for the face which uPyBot recognizes
    FACE_RECOGNITION_TIMEOUT = const(15) #number of seconds which must expire since last face recognition to recognize it again

    def __init__(self):
        self._tmr2 = Timer(2, freq=20000, mode=Timer.CENTER) #used for PWM for motor control
        self._ch1 = self._tmr2.channel(1, Timer.PWM, pin=Pin.board.X1, pulse_width=(self._tmr2.period() + 1) // 2)
        self._ch4 = self._tmr2.channel(4, Timer.PWM, pin=Pin.board.X4, pulse_width=(self._tmr2.period() + 1) // 2)
        self._us_tmr = Timer(3, prescaler=83, period=0x3fffffff) #used to measure US sensor pulse duration
        self._pin_in1 = Pin(Pin.board.X3, Pin.OUT_PP) #motor control pins
        self._pin_in2 = Pin(Pin.board.X2, Pin.OUT_PP)
        self._pin_in3 = Pin(Pin.board.X5, Pin.OUT_PP)
        self._pin_in4 = Pin(Pin.board.X6, Pin.OUT_PP) 
        self._pin_int7 = Pin(Pin.board.X7, Pin.IN) #encoder interrupt pins
        self._pin_int8 = Pin(Pin.board.X8, Pin.IN)
        self._pin_trigger = Pin(Pin.board.X11, Pin.OUT_PP) #US trig pin
        self._pin_echo = Pin(Pin.board.X12, Pin.IN) #US echo pin
        self._pin_beep = Pin(Pin.board.X19, Pin.OUT_PP) #beeper ctrl pin
        self._beeper = Beeper(self._pin_beep) 
        self._left_motor = Motor(self._pin_in1,self._pin_in2,self._ch1)
        self._left_encoder = Encoder(self,self._pin_int7)
        self._right_motor = Motor(self._pin_in4, self._pin_in3,self._ch4)
        self._right_encoder = Encoder(self,self._pin_int8)
        self._us = UltraSonicSensor(self._pin_trigger,self._pin_echo,self._us_tmr)
        self._husky = HuskyLensLibrary("SERIAL") #create HuskyLens member object to control AI camera
        self._last_hello_time = 0 #keep time when uPyBot recognized face last time
    
    def beep(self,duration = Beeper.BEEP_DEFAULT_DURATION_MS):
        self._beeper.beep(duration)

    def stop(self):
        self._left_motor.stop()
        self._right_motor.stop()
        self._left_encoder.force_stop_cnt()
        self._right_encoder.force_stop_cnt()

    def forward(self,pulses,speed): #move forward specified number of the encoder pulses with specified speed
        pyb.delay(uPyBot.NEXT_MOVE_TIMEOUT_MS) #wait before next move to avoid system overload by motors changing directions rapidly
        self._left_encoder.set_stop_pulses(pulses)
        self._right_encoder.set_stop_pulses(pulses)
        self._left_motor.set_speed(speed)
        self._right_motor.set_speed(speed)
        self._left_motor.set_forward()
        self._right_motor.set_forward()

    def backward(self,pulses,speed):#move backward specified number of the encoder pulses with specified speed
        pyb.delay(uPyBot.NEXT_MOVE_TIMEOUT_MS) #wait before next move to avoid system overload by motors changing directions rapidly
        self._left_encoder.set_stop_pulses(pulses)
        self._right_encoder.set_stop_pulses(pulses)
        self._left_motor.set_speed(speed)
        self._right_motor.set_speed(speed)
        self._left_motor.set_backward()
        self._right_motor.set_backward()

    def turn_left(self,pulses): #turn specified number of pulses
        pyb.delay(uPyBot.NEXT_MOVE_TIMEOUT_MS) #wait before next move to avoid system overload by motors changing directions rapidly
        self._left_encoder.set_stop_pulses(pulses)
        self._right_encoder.set_stop_pulses(pulses)
        self._left_motor.set_speed(uPyBot.HIGH_SPEED)
        self._right_motor.set_speed(uPyBot.HIGH_SPEED)
        self._left_motor.set_backward()
        self._right_motor.set_forward()

    def turn_right(self,pulses):
        pyb.delay(uPyBot.NEXT_MOVE_TIMEOUT_MS) #wait before next move to avoid system overload by motors changing directions rapidly
        self._left_encoder.set_stop_pulses(pulses)
        self._right_encoder.set_stop_pulses(pulses)
        self._left_motor.set_speed(uPyBot.HIGH_SPEED)
        self._right_motor.set_speed(uPyBot.HIGH_SPEED)
        self._left_motor.set_forward()
        self._right_motor.set_backward()

    def left_wheel_forward(self,pulses):
        pyb.delay(uPyBot.NEXT_MOVE_TIMEOUT_MS) #wait before next move to avoid system overload by motors changing directions rapidly
        self._left_encoder.set_stop_pulses(pulses)
        self._right_encoder.set_stop_pulses(0)
        self._left_motor.set_speed(uPyBot.HIGH_SPEED)
        self._right_motor.set_speed(0)
        self._left_motor.set_forward()
        self._right_motor.stop()
    
    def left_wheel_backward(self,pulses):
        pyb.delay(uPyBot.NEXT_MOVE_TIMEOUT_MS) #wait before next move to avoid system overload by motors changing directions rapidly
        self._left_encoder.set_stop_pulses(pulses)
        self._right_encoder.set_stop_pulses(0)
        self._left_motor.set_speed(uPyBot.HIGH_SPEED)
        self._right_motor.set_speed(0)
        self._left_motor.set_backward()
        self._right_motor.stop()
    
    def right_wheel_forward(self,pulses):
        pyb.delay(uPyBot.NEXT_MOVE_TIMEOUT_MS) #wait before next move to avoid system overload by motors changing directions rapidly
        self._left_encoder.set_stop_pulses(0)
        self._right_encoder.set_stop_pulses(pulses)
        self._left_motor.set_speed(0)
        self._right_motor.set_speed(uPyBot.HIGH_SPEED)
        self._left_motor.stop()
        self._right_motor.set_forward()
    
    def right_wheel_backward(self,pulses):
        pyb.delay(uPyBot.NEXT_MOVE_TIMEOUT_MS) #wait before next move to avoid system overload by motors changing directions rapidly
        self._left_encoder.set_stop_pulses(0)
        self._right_encoder.set_stop_pulses(pulses)
        self._left_motor.set_speed(0)
        self._right_motor.set_speed(uPyBot.HIGH_SPEED)
        self._left_motor.stop()
        self._right_motor.set_backward()

    def is_stopped(self):
        return self._left_encoder.is_stopped() and self._right_encoder.is_stopped()

    def step_back(self):
        self.backward(uPyBot.STEP_BACK_DISTANCE,uPyBot.MID_SPEED)
    
    def is_stall(self): #check if any encoder is not moving since last check what means motor stall if run
        if self._left_encoder.is_moving():
            if not self._left_encoder.is_changing():
                return True
        if self._right_encoder.is_moving():
            if not self._right_encoder.is_changing():
                return True
        return False

    def wait_until_move_done(self): #when movement is triggered and ongoing check for obstacle and motor stall and your face recognition
        start = time.ticks_ms()
        while not self.is_stopped():
            if self._us.distance_to_obstacle_in_cm() < uPyBot.LOW_DISTANCE: #detect obstacle
                self.stop()
                return uPyBot.MOVE_OBSTACLE
            if time.ticks_diff(time.ticks_ms(), start) > uPyBot.STALL_CHECK_TIME_MS: #detect motor blockage (encoder counted pulses during STALL_CHECK_TIME_MS should change)
                start = time.ticks_ms()
                if self.is_stall():
                    return uPyBot.MOVE_STALL
            if self._last_hello_time == 0: #face not recognized so far
                if self.is_face_recognized(): #check if your face (FACE_ID_TO_RECOGNIZE) is detected by Husky Lens
                    self.stop()
                    self._last_hello_time = time.time() #get current time as the moment of face recognition
                    return uPyBot.MOVE_FACE_RECOGNIZED
            elif (time.time()-self._last_hello_time) > uPyBot.FACE_RECOGNITION_TIMEOUT: #if enough time expired to recognize face again
                LED(4).on() #blue LED on means system is ready to recognize your face
                if self.is_face_recognized(): #check if your face (FACE_ID_TO_RECOGNIZE) is detected by Husky Lens
                    self.stop()
                    self._last_hello_time = time.time() #get current time as the moment of face recognition
                    return uPyBot.MOVE_FACE_RECOGNIZED
            else:
                LED(4).off() #blue LED off means system is not ready to recognize your face 
        return uPyBot.MOVE_OK

    def rnd_move(self): #make rundom distance move forward
        distance = random.randint(uPyBot.MOVE_MIN,uPyBot.MOVE_MAX)
        self.forward(distance,uPyBot.HIGH_SPEED)
        result = self.wait_until_move_done()
        if result == uPyBot.MOVE_FACE_RECOGNIZED:#when your face recognized during movement
            self.turn_for_hello() #make hello greating
        if result != uPyBot.MOVE_OK:
            self.step_back()
        return result

    def symmetric_turn(self,direction,angle):
        if direction == uPyBot.LEFT_TURN:
            self.turn_left(angle)
        else:
            self.turn_right(angle)
        result = self.wait_until_move_done()
        if result != uPyBot.MOVE_OK:
            self.step_back()

    def asymmetric_turn(self,direction,angle):
        #turn around one of the wheels
        if direction == uPyBot.LEFT_TURN:
            self.right_wheel_forward(angle)
        else:
            self.left_wheel_forward(angle)
        result = self.wait_until_move_done()
        if result != uPyBot.MOVE_OK:
            self.step_back()

    def rnd_turn(self,move_result): #make rundom turn of random type but only if not an obstacle detected for which 90 deg turn is executed always
        turn_type = random.choice([uPyBot.SYMMETRIC_TURN,uPyBot.ASYMMETRIC_TURN])
        turn_direction = random.choice([uPyBot.LEFT_TURN,uPyBot.RIGHT_TURN])
        turn_angle = random.choice([uPyBot.TURN_90,uPyBot.TURN_180])
        wheel_turn_angle = random.choice([uPyBot.WHEEL_TURN_90,uPyBot.WHEEL_TURN_180])
        if move_result != uPyBot.MOVE_OK:
            self.symmetric_turn(turn_direction,uPyBot.TURN_90)
        else:
            if turn_type == uPyBot.SYMMETRIC_TURN:
                self.symmetric_turn(turn_direction,turn_angle)
            else:
                self.asymmetric_turn(turn_direction,wheel_turn_angle)
   
    def turn_for_hello(self): #make hello movement used to greet when husky lens recognize FACE_ID (my own face)
        turn_direction = random.choice([uPyBot.LEFT_TURN,uPyBot.RIGHT_TURN])
        for repeat_cnt in range(uPyBot.HELLO_TURN_CNT):
            self.beep()
            if turn_direction == uPyBot.LEFT_TURN:
                self.symmetric_turn(uPyBot.LEFT_TURN,uPyBot.TURN_45)
                self.symmetric_turn(uPyBot.RIGHT_TURN,uPyBot.TURN_45)
                self.symmetric_turn(uPyBot.RIGHT_TURN,uPyBot.TURN_45)
                self.symmetric_turn(uPyBot.LEFT_TURN,uPyBot.TURN_45)   
            else:
                self.symmetric_turn(uPyBot.RIGHT_TURN,uPyBot.TURN_45)
                self.symmetric_turn(uPyBot.LEFT_TURN,uPyBot.TURN_45)
                self.symmetric_turn(uPyBot.LEFT_TURN,uPyBot.TURN_45)
                self.symmetric_turn(uPyBot.RIGHT_TURN,uPyBot.TURN_45)            

    def turn_face_recognition_on(self):
        LED(4).on() #blue LED on means system is ready to recognize your face
        self._husky.face_recognition_mode()

    def is_face_recognized(self):
        result=self._husky.command_request_blocks_by_id(uPyBot.FACE_ID_TO_RECOGNIZE)
        if result:
            return True
        else:
            return False

#simple program to check various robot functions/movements
print("Start random walk program") 
robot = uPyBot()
robot.beep()     #make short beep to signal robot start
pyb.delay(2000)
robot.turn_face_recognition_on()

while True: 
    result=robot.rnd_move() #make a move
    pyb.delay(1000)
    robot.rnd_turn(result) #turn based on last move result
    pyb.delay(1000)


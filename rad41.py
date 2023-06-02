import RPi.GPIO as GPIO
from time import sleep

import cv2
import numpy as np
from picamera import PiCamera
from picamera.array import PiRGBArray

import signal

# setup pins 

in1=24
in2=23
enA=25

in21 = 17
in22 = 27
en2A = 22

in23 = 2
in24 = 3
en2B = 4

in3=5
in4=6
enB=26

moist=16

# setup gpio

GPIO.setmode(GPIO.BCM)

GPIO.setup(moist,GPIO.IN)

GPIO.setup(in1,GPIO.OUT)
GPIO.setup(in2,GPIO.OUT)
GPIO.setup(enA,GPIO.OUT)

GPIO.setup(in3,GPIO.OUT)
GPIO.setup(in4,GPIO.OUT)
GPIO.setup(enB,GPIO.OUT)

GPIO.setup(in21,GPIO.OUT)
GPIO.setup(in22,GPIO.OUT)
GPIO.setup(en2A,GPIO.OUT)

GPIO.setup(in23,GPIO.OUT)
GPIO.setup(in24,GPIO.OUT)
GPIO.setup(en2B,GPIO.OUT)

GPIO.output(in1, GPIO.LOW)
GPIO.output(in2, GPIO.LOW)

GPIO.output(in3, GPIO.LOW)
GPIO.output(in4, GPIO.LOW)

GPIO.output(in21, GPIO.LOW)
GPIO.output(in22, GPIO.LOW)

GPIO.output(in23, GPIO.HIGH)
GPIO.output(in24, GPIO.LOW)

# setup power

p1=GPIO.PWM(enA,1000)
p1.start(25)

p2=GPIO.PWM(enB,1000)
p2.start(100)

p3=GPIO.PWM(en2A,1000)
p3.start(25)

p4=GPIO.PWM(en2B,1000)
p4.start(25)

STOP=1
FORWARD=2
BACKWARD=3
CLEANUP=7

SPRINKLE=8

# setup main power control for movement, spriknling, and motor of moist sensor

def set_movement_type(type):
    if type==STOP:
        GPIO.output(in1, GPIO.LOW)
        GPIO.output(in2, GPIO.LOW)
    elif type==FORWARD:
        GPIO.output(in1, GPIO.HIGH)
        GPIO.output(in2, GPIO.LOW)
    elif type==BACKWARD:
        GPIO.output(in1, GPIO.LOW)
        GPIO.output(in2, GPIO.HIGH)
        
def set_sprinkle_type(type):
    if type==STOP:
        GPIO.output(in3, GPIO.LOW)
        GPIO.output(in4, GPIO.LOW)
    elif type==SPRINKLE:
        GPIO.output(in3, GPIO.HIGH)
        GPIO.output(in4, GPIO.LOW)
        
def set_motor_type(type):
    if type==STOP:
        GPIO.output(in21, GPIO.LOW)
        GPIO.output(in22, GPIO.LOW)
    elif type==FORWARD:
        GPIO.output(in21, GPIO.HIGH)
        GPIO.output(in22, GPIO.LOW)
    elif type==BACKWARD:
        GPIO.output(in21, GPIO.LOW)
        GPIO.output(in22, GPIO.HIGH)


# cleanup function
def cleanup():
    GPIO.cleanup()
    exit(3)

# cleanup for ctrl + c
signal.signal(signal.SIGINT, cleanup)

# helper functions for better coding
def get_moist_lvl():
    return GPIO.input(moist)


def get_water_level():
    inputs = []
    for i in range(10):
        inputs.append(get_moist_lvl())
        sleep(0.1)
    lvl = sum(inputs) / 10 
    print("Watering level is ", lvl * 100, "%")
    return lvl

def is_watered():
    return get_water_level() >= 0.7

def move_forward():
    set_movement_type(FORWARD)
    
def move_backward():
    set_movement_type(BACKWARD)
    
def stop():
    set_movement_type(STOP)

def motor(f=True):
    if f:
        set_motor_type(FORWARD)
    else:
        set_motor_type(BACKWARD)

def motor_stop():
    set_motor_type(STOP)

def sprinkle(f=True):
    if f:
        set_sprinkle_type(SPRINKLE)
    else:
        set_sprinkle_type(STOP)

# water main function. used for watering plant algorithm
def water():
    print("STOP")
    set_movement_type(STOP)
    sleep(1)
    
    set_motor_type(FORWARD)
    print("MOIST DOWN")
    sleep(15)
    set_motor_type(STOP)
    print("MOIST STOP")
    print("SPRINKLE")
#     until watered, sprinkle
    while not is_watered():
        print("SPRINKLE")
        sprinkle()
        sleep(1)
    set_sprinkle_type(STOP)
    set_motor_type(BACKWARD)
    print("MOIST UP")
    sleep(15)
    set_motor_type(STOP)
    print("MOIST STOP")
    set_movement_type(BACKWARD)
    print("MOVE BACK")
    sleep(5)
    set_movement_type(STOP)
    print("MOVE STOP")
#     cooldown for cycle
    print("CYCLE DONE! (cooldown 10 sec)")
    sleep(10)
    
    
    

if __name__=='__main__':
    while True:
#         setup camera
        camera = PiCamera()
        camera.resolution = (640, 480)
        camera.framerate = 15
        rawCapture = PiRGBArray(camera, size=(640,480))
        pixels = 640 * 480
        
#         for every frame of camera (15 times/second)
        for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
            frame = frame.array
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            green_lower = np.array([20, 40, 60])
            green_upper = np.array([102, 255, 255])
            green_mask = cv2.inRange(hsv, green_lower, green_upper)
            result_green = cv2.bitwise_and(frame, frame, mask=green_mask)
            
            # this is used to show the result on the screen
            cv2.imshow('', result_green)
            
            rawCapture.truncate(0)
            
            # process q key to quit in camera
            if cv2.waitKey(5) & 0xFF == ord('q'):
                cleanup()
                camera.close()
            
#             if the amount of green pixels > 70% -> water the plant
            if cv2.countNonZero(green_mask)>(0.7*pixels):
                print("Green detected")
                water()
#             if the amount is > 20% -> move towards the plant
            elif cv2.countNonZero(green_mask)>(0.2*pixels):
                print("Green detected", 100 * (cv2.countNonZero(green_mask)) / pixels, "%")
                set_movement_type(FORWARD)
#             othervise wait for green color (for plant)
            else:
                print("No green :(")
                set_movement_type(STOP)

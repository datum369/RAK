import socket

import RPi.GPIO as GPIO
import time
from gpiozero import AngularServo

from track import pick

HOST ="0.0.0.0"  # Listen on all interfaces
PORT =5000

# Define GPIO pins for motor driver control
in1 = 17  # Left motor forward
in2 = 27  # Left motor backward
in3 = 22  # Right motor forward
in4 = 23  # Right motor backward
enA = 24  # Left motor enable
enB = 25  # Right motor enable

# Servos driving
servo2 = AngularServo(17, min_angle=0, max_angle=270, min_pulse_width=0.0006, max_pulse_width=0.0023)
servo3 = AngularServo(27, min_angle=0, max_angle=270, min_pulse_width=0.0006, max_pulse_width=0.0023)
#servo4 = AngularServo(27, min_angle=0, max_angle=270, min_pulse_width=0.0006, max_pulse_width=0.0023)
clamper = AngularServo(22, min_angle=0, max_angle=90, min_pulse_width=0.0006, max_pulse_width=0.0023)

servo2=16
servo3=20
clamper=21

clamper_open_time = 2  # Time to keep clamper open (in seconds)
clamper_close_time = 10 # Time to keep clamper closed (in seconds)

clamper_open = True  # Flag to indicate whether the clamper is currently open or closed
clamper_last_toggle_time = time.time()  # Initialize last toggle time

# Set GPIO mode to BCM
GPIO.setmode(GPIO.BCM)

# Set GPIO pins as output
GPIO.setup(in1, GPIO.OUT)
GPIO.setup(in2, GPIO.OUT)
GPIO.setup(in3, GPIO.OUT)
GPIO.setup(in4, GPIO.OUT)
GPIO.setup(enA, GPIO.OUT)
GPIO.setup(enB, GPIO.OUT)


def forward():
    GPIO.output(in1, GPIO.HIGH)
    GPIO.output(in2, GPIO.LOW)
    GPIO.output(in3, GPIO.HIGH)
    GPIO.output(in4, GPIO.LOW)
    time.sleep(2)
    GPIO.output(in1, GPIO.LOW)
    GPIO.output(in2, GPIO.LOW)
    GPIO.output(in3, GPIO.LOW)
    GPIO.output(in4, GPIO.LOW)
    

def backward():
    GPIO.output(in1, GPIO.LOW)
    GPIO.output(in2, GPIO.HIGH)
    GPIO.output(in3, GPIO.LOW)
    GPIO.output(in4, GPIO.HIGH)
    time.sleep(2)
    GPIO.output(in1, GPIO.LOW)
    GPIO.output(in2, GPIO.LOW)
    GPIO.output(in3, GPIO.LOW)
    GPIO.output(in4, GPIO.LOW)

def pick():
    servo2.angle=90
    servo3.angle=234
    current_time = time.time()
    if clamper_open and current_time - clamper_last_toggle_time >= clamper_open_time:
        clamper.angle = 0  # Close clamper
        clamper_open = False
        clamper_last_toggle_time = current_time
    elif not clamper_open and current_time - clamper_last_toggle_time >= clamper_close_time:
        clamper.angle = 90  # Open clamper
        clamper_open = True
        clamper_last_toggle_time = current_time
	
	
	

def right():
    GPIO.output(in1, GPIO.LOW)
    GPIO.output(in2, GPIO.LOW)
    GPIO.output(in3, GPIO.HIGH)
    GPIO.output(in4, GPIO.LOW)
    time.sleep(2)
    GPIO.output(in1, GPIO.LOW)
    GPIO.output(in2, GPIO.LOW)
    GPIO.output(in3, GPIO.LOW)
    GPIO.output(in4, GPIO.LOW)

def left():
    GPIO.output(in1, GPIO.HIGH)
    GPIO.output(in2, GPIO.LOW)
    GPIO.output(in3, GPIO.LOW)
    GPIO.output(in4, GPIO.LOW)
    time.sleep(3)
    GPIO.output(in1, GPIO.LOW)
    GPIO.output(in2, GPIO.LOW)
    GPIO.output(in3, GPIO.LOW)
    GPIO.output(in4, GPIO.LOW)

def stop():
    GPIO.output(in1, GPIO.LOW)
    GPIO.output(in2, GPIO.LOW)
    GPIO.output(in3, GPIO.LOW)
    GPIO.output(in4, GPIO.LOW)
    time.sleep(2)
    GPIO.output(in1, GPIO.LOW)
    GPIO.output(in2, GPIO.LOW)
    GPIO.output(in3, GPIO.LOW)
    GPIO.output(in4, GPIO.LOW)


# You can use PWM to control motor speed if needed
pwmA = GPIO.PWM(enA, 100)  # Set PWM frequency to 50 Hz
pwmB = GPIO.PWM(enB, 100)
pwmA.start(100)  # Set initial duty cycle to 50%
pwmB.start(100)


sock=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
sock.bind((HOST,PORT))
sock.listen(1)

print(f"Listening for connection on {HOST}:{PORT}")
conn,addr = sock.accept()
print(f"Connection from {addr}")

while True:
    data = conn.recv(1024)
    if not data:
        break
        
    received_text=data.decode()
    print(f"Recieved Text:{received_text}")
    if "forward" in received_text:
        print("forward")
        forward()
    if "backward" in received_text:
        print("backward")
        backward()
    if "left" in received_text:
        print("left")
        left()
    if "right" in received_text:
        print("right")
        right()
        
    
conn.close()

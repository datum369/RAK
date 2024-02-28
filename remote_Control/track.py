import cv2
from gpiozero import AngularServo
import numpy as np
import time

# Servos driving
servo2 = AngularServo(17, min_angle=0, max_angle=270, min_pulse_width=0.0006, max_pulse_width=0.0023)
servo3 = AngularServo(27, min_angle=0, max_angle=270, min_pulse_width=0.0006, max_pulse_width=0.0023)
#servo4 = AngularServo(27, min_angle=0, max_angle=270, min_pulse_width=0.0006, max_pulse_width=0.0023)
clamper = AngularServo(22, min_angle=0, max_angle=90, min_pulse_width=0.0006, max_pulse_width=0.0023)

# Initialize previous trackbar values
prev_values = [0, 90, 0]

def smooth_values(current_values, prev_values, alpha=0.5):
    """
    Smooths the current trackbar values using exponential moving average.
    """
    smoothed_values = []
    for current_val, prev_val in zip(current_values, prev_values):
        smoothed_val = alpha * current_val + (1 - alpha) * prev_val
        smoothed_values.append(int(smoothed_val))
    return smoothed_values

def on_trackbar(val):
    print("Trackbar value:", val)
    return val

cv2.namedWindow('TrackBars')
cv2.resizeWindow('TrackBars', 1200, 600)
cv2.createTrackbar("Servo 2", "TrackBars", 90, 180, on_trackbar)
cv2.createTrackbar("Servo 3", "TrackBars", 0, 180, on_trackbar)
cv2.createTrackbar("ClamperTime", "TrackBars", 2, 20, on_trackbar)
#cv2.createTrackbar("Servo 3", "TrackBars", 0, 270, on_trackbar)

clamper_open_time = 2  # Time to keep clamper open (in seconds)
clamper_close_time = cv2.getTrackbarPos('ClamperTime', "TrackBars")  # Time to keep clamper closed (in seconds)

clamper_open = True  # Flag to indicate whether the clamper is currently open or closed
clamper_last_toggle_time = time.time()  # Initialize last toggle time


def pick():
    while True:
        # Get current trackbar values
        #dof1 = cv2.getTrackbarPos('Servo 2', 'TrackBars')
        dof2 = cv2.getTrackbarPos('Servo 2', 'TrackBars')
        dof3 = cv2.getTrackbarPos('Servo 3', 'TrackBars')
        clamper_close_time = cv2.getTrackbarPos('ClamperTime', "TrackBars") 
        # Time to keep clamper closed (in seconds)
        

        # Smooth the current trackbar values
        smoothed_values = smooth_values([dof2, dof3], prev_values)
        dof2, dof3 = smoothed_values

        print(dof2, dof3, )

        # Set servo angles
        #servo1.angle = dof1
        servo2.angle = dof2 +90
        servo3.angle = dof3 + 90

        # Update previous values
        prev_values = smoothed_values

        # Check if it's time to toggle the clamper
        current_time = time.time()
        if clamper_open and current_time - clamper_last_toggle_time >= clamper_open_time:
            clamper.angle = 0  # Close clamper
            clamper_open = False
            clamper_last_toggle_time = current_time
        elif not clamper_open and current_time - clamper_last_toggle_time >= clamper_close_time:
            clamper.angle = 90  # Open clamper
            clamper_open = True
            clamper_last_toggle_time = current_time

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

# Release resources
cv2.destroyAllWindows()

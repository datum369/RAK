from gpiozero import AngularServo
import time 
import math


# ---------------- PI-config -------------------
servo1 = AngularServo(5,min_angle=0, max_angle=270, min_pulse_width=0.0006, max_pulse_width=0.0023)
servo2 = AngularServo(6,min_angle=0, max_angle=270, min_pulse_width=0.0006, max_pulse_width=0.0023)
servo3 = AngularServo(13,min_angle=0, max_angle=270, min_pulse_width=0.0006, max_pulse_width=0.0023)


# ---------------- Arm constants and config --------------
deg = 57.29577 # def = rad * (180/pi = 57.29277)
arm = 15
hand = 14
f_arm = hand + 15 -2

# position of object
x = 16
y = 14




# ------------------ Kinematics ----------------------
def kinematics():
    # distance of point (x,y) from origin
    D = math.sqrt((x**2)+(y**2))
    print(D)

    # calculation of DOF
    dof_list = []

    dof1 = (math.atan2(y,x))*deg
    dof_list.append(dof1)

    cos_dof2 = (arm**2 + D **2 - f_arm**2)/(2*arm*D)
    dof2 = (math.acos(cos_dof2))*deg
    dof_list.append(dof2)

    # sin_dof3 = D * math.sin(dof2)/f_arm
    cos_dof3  = (arm**2 + f_arm **2 - D**2)/(2*arm*f_arm)
    dof3 = (math.acos(cos_dof3))*deg
    dof_list.append(dof3)

    # sin_dof4 = arm * math.sin(dof2)/f_arm
    # cos_dof4 = (D**2 + f_arm**2 - arm**2)/(2*f_arm*D)
    # dof4 = (math.asin(cos_dof4))*deg
    # dof4 = 180 - dof3 -dof2


    # print( "D: ",D," dof1: ", dof1," dof2: ", dof2," dof3: ", dof3, " dof4: ", dof4 )
    print(dof_list)

    # for i in range(6):
    print("\n -------> iteration: <---------")

    time.sleep(2)
    print("base motor: dof1: ", dof1)
    servo1.angle = dof1

    time.sleep(2)
    print("shoulder motor: dof2: ", dof2)
    servo2.angle = dof2

    time.sleep(2)
    print("elbow motor: dof3: ", 270-dof3)
    servo3.angle = 270-dof3

# try:
# 	while True:
# 	    kinematics()	
		

# except KeyboardInterrupt:
# 	print("Program stopped")
    
kinematics()
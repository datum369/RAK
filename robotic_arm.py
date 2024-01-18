import pyfirmata as pf
import math
import time



port = '/dev/cu.usbmodem101'
board = pf.Arduino(port)

deg = 57.29577 # def = rad * (180/pi = 57.29277)
arm = 15
hand = 14
f_arm = hand + 15 -2
# position of object
x = 16
y = 14


# motor pins
for pin in range(8,11,1):
    board.digital[pin].mode = pf.SERVO 


def kinematics():
    # distance of point (x,y) from origin
    D = math.sqrt((x**2)+(y**2))
    print(D)

    # calculation of DOF
    dof_list = []

    dof1 = (math.atan(y/x))*deg
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
    dof4 = 180 - dof3 -dof2


    # print( "D: ",D," dof1: ", dof1," dof2: ", dof2," dof3: ", dof3, " dof4: ", dof4 )
    print(dof_list)

    for i in range(6):
        print("\n -------> iteration: ", i, " <---------");

        time.sleep(2)
        print("base motor: dof1: ", dof1-i*5)
        board.digital[8].write(dof1-i*5)

        time.sleep(2)
        print("shoulder motor: dof2: ", dof2+i*15)
        board.digital[9].write(dof2+i*15)

        time.sleep(2)
        print("elbow motor: dof3: ", 270-dof3-i*20)
        board.digital[10].write(270-dof3-i*20)

    


kinematics()
import math
import cv2
import time
import numpy as np
# from gpiozero import AngularServo


cap=cv2.VideoCapture(0)


deg = 57.29577 # def = rad * (180/pi = 57.29277)
arm = 15
hand = 14
f_arm = hand + 15 -2
# position of object

# motor pins
servo1 = AngularServo(5,min_angle=0, max_angle=270, min_pulse_width=0.0006, max_pulse_width=0.0023)
servo2 = AngularServo(6,min_angle=0, max_angle=270, min_pulse_width=0.0006, max_pulse_width=0.0023)
servo3 = AngularServo(13,min_angle=0, max_angle=270, min_pulse_width=0.0006, max_pulse_width=0.0023)

# position of object
# taking reference point: (22,85)
x = 22
y = 85

def kinematics():
    # distance of point (x,y) from origin
    D = math.sqrt((x**2-22**2)+(y**2-85**2))
    print(D)

    dof1 = (math.atan2(y/x))*deg

    cos_dof2 = (arm**2 + D **2 - f_arm**2)/(2*arm*D)
    dof2 = (math.acos(cos_dof2))*deg

    # sin_dof3 = D * math.sin(dof2)/f_arm
    cos_dof3  = (arm**2 + f_arm **2 - D**2)/(2*arm*f_arm)
    dof3 = (math.acos(cos_dof3))*deg


    # sin_dof4 = arm * math.sin(dof2)/f_arm
    # cos_dof4 = (D**2 + f_arm**2 - arm**2)/(2*f_arm*D)
    # dof4 = (math.asin(cos_dof4))*deg

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


def getContours(masked_img, real_img):
    m_img = np.array(masked_img, np.uint8)
    contours, _= cv2.findContours(m_img, cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
    area = 0
    for cnt in contours:
        area=cv2.contourArea(cnt)
        if area >=900.0:
            
            cv2.drawContours(m_img,cnt,-1,(255,0,0),3)
            M = cv2.moments(cnt)

            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])

            

            cv2.circle(real_img, (cx,cy),4,(255,0,0), -1)
            centre_text = "( " + str(cx) + " , " + str(cy) + " )"
            cv2.putText(real_img, centre_text , (cx-20, cy-20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,0),1)     
            result=cv2.bitwise_and(real_img, real_img, mask=m_img)
            
            cv2.imshow("real_img", real_img)
            cv2.imshow("m_img", result)

    
    print("Area: ", area, "\n")
    print("(cx, cy): (", x, " , ", y, " )")
    x=cx
    y=cy
    kinematics()
    



def empty(a):
  pass

cv2.namedWindow('TrackBars')
cv2.resizeWindow('TrackBars',640,240)
cv2.createTrackbar("Hue Min","TrackBars",16,179,empty)
cv2.createTrackbar("Hue Max","TrackBars",35,179,empty)
cv2.createTrackbar("Sat Min","TrackBars",56,255,empty)
cv2.createTrackbar("Sat Max","TrackBars",255,255,empty)
cv2.createTrackbar("Val Min","TrackBars",139,255,empty)
cv2.createTrackbar("Val Max","TrackBars",255,255,empty)



while True:
  success,img=cap.read()
  HSV_img=cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
  stak=np.hstack((img,HSV_img))
  h_min=cv2.getTrackbarPos('Hue Min','TrackBars')
  h_max=cv2.getTrackbarPos('Hue Max','TrackBars')
  s_min=cv2.getTrackbarPos('Sat Min','TrackBars')
  s_max=cv2.getTrackbarPos('Sat Max','TrackBars')
  v_min=cv2.getTrackbarPos('Val Min','TrackBars')
  v_max=cv2.getTrackbarPos('Val Max','TrackBars')
  print(h_min,h_max,s_min,s_max,v_min,v_max)
  lower=np.array([h_min,s_min,v_min])
  upper=np.array([h_max,s_max,v_max])
  mask = cv2.inRange(HSV_img,lower,upper)
  
  getContours(mask, img)

#   result=cv2.bitwise_and(img,img,mask=mask)
#   cv2.imshow('output',stak)
#   cv2.imshow("mask",mask)
#   cv2.imshow('result image',result)
  if cv2.waitKey(1) & 0xFF==ord('q'):
    break




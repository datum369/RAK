import cv2
import numpy as np
import math



# from gpiozero import AngularSerqvo;
# servo1 = AngularServo(38,min_angle=0, max_angle=270, min_pulse_width=0.0006, max_pulse_width=0.0023)
# servo2 = AngularServo(21,min_angle=0, max_angle=270, min_pulse_width=0.0006, max_pulse_width=0.0023)
# servo3 = AngularServo(12,min_angle=0, max_angle=270, min_pulse_width=0.0006, max_pulse_width=0.0023)


cap=cv2.VideoCapture(0)
cap.set(3,640)
cap.set(4,480)
cap.set(10,150)

def getContours(masked_img, real_img):
    m_img = np.array(masked_img, np.uint8)
    contours, _= cv2.findContours(m_img, cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
    area = 0
    cx,cy = 0,0

    for cnt in contours:
        area=cv2.contourArea(cnt)
        # print(area)
        #print(area)
        if area >=900.0:
            

            cv2.drawContours(m_img,cnt,-1,(255,0,0),3)
            M = cv2.moments(cnt)

            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            

            cv2.circle(real_img, (cx,cy),4,(255,0,0), -1)
            centre_text = "( " + str(cx) + " , " + str(cy) + " )"
            cv2.putText(real_img, centre_text ,(int(2*cx),int(2*cy)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0),1)     
            result=cv2.bitwise_and(real_img, real_img, mask=m_img)

            cv2.imshow("real_img", real_img)
            cv2.imshow("m_img", result)

    
    print("Area: ", area, "\n")
    print("(cx, cy): (", cx, " , ", cy, " )")
    return (cx, cy)

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
  
  x_mid, y_mid = getContours(mask, img)
  x_mid, y_mid = int(x_mid), int(y_mid)
  X_distance = math.hypot(x_mid-0)
  Y_distance = math.hypot(y_mid-480)
  print(X_distance)
  cv2.line(img,(0,480),(x_mid,y_mid),(0,0,255),5)
  cv2.line(img,(640,480),(x_mid,y_mid),(0,0,255),5)
  X_distance=np.interp(X_distance,[0,600],[0,180]) #this line changes the distance into degrees of rotation
  Y_distance=np.interp(Y_distance,[0,450],[0,180]) #this line changes the distance into degrees of rotation
  print("X_distance is :",X_distance)
  # servo1.angle = X_distance
  print("Y_distance is :",Y_distance)
  # servo2.angle = 100
  # servo3.angle = 200
  
  print("Returned: ", x_mid, y_mid)
  result=cv2.bitwise_and(img,img,mask=mask)

  # cv2.imshow('output',stak)
  # cv2.imshow("mask",mask)
  # cv2.imshow('result image',result)
  if cv2.waitKey(1) & 0xFF==ord('q'):
    break
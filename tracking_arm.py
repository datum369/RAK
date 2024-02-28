import cv2
import math
import numpy as np
from gpiozero import AngularServo;


# Servos driving
servo1 = AngularServo(17,min_angle=0, max_angle=270, min_pulse_width=0.0006, max_pulse_width=0.0023)
servo2 = AngularServo(27,min_angle=0, max_angle=270, min_pulse_width=0.0006, max_pulse_width=0.0023)
servo3 = AngularServo(22,min_angle=0, max_angle=270, min_pulse_width=0.0006, max_pulse_width=0.0023)


cap=cv2.VideoCapture(0)
cap.set(10,150)
lower_range=np.array([24,112,73])
upper_range=np.array([165,255,255])


Known_distance = 13.0 #cm
Known_width = 3.0 #cm

def Focal_Length_Finder(Known_distance, real_width, width_in_rf_image):
    focal_length = (width_in_rf_image * Known_distance) / real_width
    return focal_length

def obj_data(img):
     obj_width = 0
     hsv=cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
     mask=cv2.inRange(hsv,lower_range,upper_range)
     _,mask1=cv2.threshold(mask,254,255,cv2.THRESH_BINARY)
     cnts,_=cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
     for c in cnts:
        x=600
        if cv2.contourArea(c)>x:
            x,y,w,h=cv2.boundingRect(c)
            cv2.rectangle(img,(x,y),(x+w,y+h),(0,255,0),2)
            obj_width = w
            
     return obj_width

def get_mid(img):
    x_mid,y_mid=0,0
    hsv=cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
    mask=cv2.inRange(hsv,lower_range,upper_range)
    _,mask1=cv2.threshold(mask,254,255,cv2.THRESH_BINARY)
    contours,hierarchy = cv2.findContours(mask1,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
    x,y,w,h=0,0,0,0
    # left_distance,right_distance=0,0
    for cnt in contours:
        area=cv2.contourArea(cnt)
        
            
     
        cv2.drawContours(img,cnt,-1,(255,0,0),3)
        peri=cv2.arcLength(cnt,True)
        #print(peri)
        approx=cv2.approxPolyDP(cnt,0.02*peri,True) 
        objCOr=len(approx)
        x,y,w,h=cv2.boundingRect(approx)
        # cv2.rectangle(img,(x,y),(x+w,y+h),(0,255,0),5)
        cv2.circle(img,(x+w//2,y+h//2),5,(255,0,0),-1)
        x_mid,y_mid=x+w//2,y+h//2
        cv2.circle(img,(0,190),10,(0,255,0),-1)
        # cv2.line(img,(0,190),(x_mid,y_mid),(0,255,0),5)
        

    return x_mid,y_mid


def Distance_finder(Focal_Length, Known_width, obj_width_in_frame):
    distance = (Known_width * Focal_Length)/obj_width_in_frame
    return distance    

ref_image = cv2.imread("rf.png")
ref_image_obj_width = obj_data(ref_image)
Focal_length_found = Focal_Length_Finder(Known_distance, Known_width, ref_image_obj_width)
cv2.imshow("ref_image", ref_image)

print(Focal_length_found)


while True:
    ret,frame=cap.read()
    frame=cv2.resize(frame,(640,480))
    obj_width_in_frame=obj_data(frame)
    x_mid,y_mid= get_mid(frame)
    X_distance = math.hypot(x_mid-0)
    Y_distance = math.hypot(y_mid-480)
    print(X_distance)
    cv2.line(frame,(0,480),(x_mid,y_mid),(0,0,255),5)
    cv2.line(frame,(640,480),(x_mid,y_mid),(0,0,255),5)
    X_distance=np.interp(X_distance,[0,600],[0,180]) #this line changes the distance into degrees of rotation
    Y_distance=np.interp(Y_distance,[0,450],[0,180]) #this line changes the distance into degrees of rotation
    print("X_distance is :",X_distance)
    servo1.angle = X_distance
    print("Y_distance is :",Y_distance)
    servo2.angle = 90
    servo3.angle = 180

    if obj_width_in_frame != 0:
        Distance = Distance_finder(Focal_length_found, Known_width, obj_width_in_frame)
        cv2.putText(frame, f"Distance: {round(Distance,2)} CM", (30, 35),cv2.FONT_HERSHEY_COMPLEX, 0.6, (255,0,0), 2)
   
    
          
            
    cv2.imshow("FRAME",frame)
    if cv2.waitKey(1)&0xFF==27:
        break
cap.release()
cv2.destroyAllWindows()

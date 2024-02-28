import math
import cv2
import numpy as np
#from gpiozero import AngularServo


cap=cv2.VideoCapture(0)
cap.set(3,640)
cap.set(4,480)
#cap.set(10,150)


# -------- 
# dist = np.sqrt(640**2+480**2)
# ppi = (dist/13.6) 
# d = np.sqrt(22**2+ 28**2)
# dinch = d/ppi
# dcm = dinch*2.54

deg = 57.29577 # def = rad * (180/pi = 57.29277)

arm = 12 # length of arm in cm
farm = 12 # length of forearm in cm
bh = 10.5 # height of base motor in cm

# position of object
# motor pins
#servo1 = AngularServo(17,min_angle=0, max_angle=270, min_pulse_width=0.0006, max_pulse_width=0.0023)
#servo2 = AngularServo(16,min_angle=0, max_angle=270, min_pulse_width=0.0006, max_pulse_width=0.0023)
#servo3 = AngularServo(20,min_angle=0, max_angle=270, min_pulse_width=0.0006, max_pulse_width=0.0023)


# --------------- distanc setup --------------
h_min,h_max,s_min,s_max,v_min,v_max = 16, 35, 56, 255, 139, 255

lower=np.array([h_min,s_min,v_min])
upper=np.array([h_max,s_max,v_max])

Known_distance = 13.0 #cm
Known_width = 5.0 #cm

def Focal_Length_Finder(Known_distance, real_width, width_in_rf_image):
    focal_length = (width_in_rf_image * Known_distance) / real_width
    return focal_length
  

def obj_data(mask, img):
    obj_width = 0
    center_x,center_y=0,0
    #  hsv=cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
    #  mask=cv2.inRange(hsv,lower_range,upper_range)
    #  _,mask1=cv2.threshold(mask,254,255,cv2.THRESH_BINARY)
    cnts,_=cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
    for con in cnts:
      x=600
      if cv2.contourArea(con)>x:
        x,y,w,h=cv2.boundingRect(con)
        cv2.rectangle(img,(x,y),(x+w,y+h),(0,255,0),2)
        obj_width = w
        center_x=(x+w)//2
        center_y=(y+h)//2
        cv2.circle(img,(center_x,center_y),5,(0,255,0),-1)
    return obj_width

def obj_data_1(mask):
    obj_width = 0
    center_x,center_y=0,0
    #  hsv=cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
    #  mask=cv2.inRange(hsv,lower_range,upper_range)
    #  _,mask1=cv2.threshold(mask,254,255,cv2.THRESH_BINARY)
    cnts,_=cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
    for con in cnts:
      x=600
      if cv2.contourArea(con)>x:
        x,y,w,h=cv2.boundingRect(con)
        # cv2.rectangle(img,(x,y),(x+w,y+h),(0,255,0),2)
        obj_width = w
        # center_x=(x+w)//2
        # center_y=(y+h)//2
        # cv2.circle(img,(center_x,center_y),5,(0,255,0),-1)
    return obj_width

def Distance_finder(Focal_Length, Known_width, obj_width_in_frame):
    distance = (Known_width * Focal_Length)/obj_width_in_frame
    return distance 

path_img_pi = "/home/pi/Desktop/our_env/major_project/Bogati/rf.png"
path_img_mac = "/Users/eudgen/Desktop/Robotic Arm/RA_code/front_dist_approach/rf.png"
ref_image = cv2.imread(path_img_mac)
#ref_image = cv2.imread(path_img_pi)
HSVimg=cv2.cvtColor(ref_image,cv2.COLOR_BGR2HSV)
mask_img = cv2.inRange(HSVimg,lower,upper)
ref_image_obj_width = obj_data_1(mask_img)
Focal_length_found = Focal_Length_Finder(Known_distance, Known_width, ref_image_obj_width)
cv2.imshow("ref_image", ref_image)

print(Focal_length_found)


def kinematics(dcm):
  
    # d_pix = np.sqrt((x**2-2**2)+(y**2-4**2)) # distance in pixel coordinate
    # d_pixInc = d_pix/ppi # convert to inch
    # Distance from base motor centre to object  in cm
    if dcm <13:
       d_cm = dcm
    else:
       d_cm = dcm + 10
    
    d_sl = np.sqrt((d_cm**2)+(bh**2))  # slated distance from top of base motor to position of object
    print("\nDistance base to object : ", d_cm ," cm")

    #------------ dof1 ----------------
    # dof1 = (math.atan2(y, x))*deg
    # print("dof1: ", dof1)
    
    
    #------------ dof2 ----------------
    cos_dof2 = (arm**2+d_sl**2 - farm**2)/(2*arm*d_sl)
    print("cos_dof2: ",cos_dof2)
    

    #------------ dof3 ----------------
    # sin_dof3 = D * math.sin(dof2)/f_arm
    cos_dof3  = (arm**2+farm**2-d_sl**2)/(2*arm*farm)
    print("cos_dof3: ",cos_dof3)
    
    dof2_case = (cos_dof2<=1 and cos_dof2>=-1)
    dof3_case = (cos_dof3<=1 and cos_dof3>=-1)
    if(dof2_case and dof3_case):
		
      dof2 = (math.acos(cos_dof2))*deg
      dof3 = 180 - (math.acos(cos_dof3))*deg
      
      bol_dof2 = dof2>0 and dof2<270
      bol_dof3 = dof3>0 and dof3<270
      if (bol_dof2 and bol_dof3):
        # sin_dof4 = arm * math.sin(dof2)/f_arm
        # cos_dof4 = (D**2 + f_arm**2 - arm**2)/(2*f_arm*D)
        # dof4 = (math.asin(cos_dof4))*deg

        print("\n -------> Applying angles to servos  <---------")

        # time.sleep(2)
        # print("base motor: dof1: ", dof1)
        # servo1.angle = dof1

        # time.sleep(2)
        print("shoulder motor: dof2: ", dof2)
        #servo2.angle = dof2

        # time.sleep(2)
        print("elbow motor: dof3: ", dof3)
        #servo3.angle = dof3

    else:
       pass
      



# def getContours(masked_img, real_img):
#     m_img = np.array(masked_img, np.uint8)
#     contours, _= cv2.findContours(m_img, cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
#     area = 0
#     cx, cy = 0,0
#     for cnt in contours:
#         area=cv2.contourArea(cnt)
#         if area >=900.0:
            
#             cv2.drawContours(m_img,cnt,-1,(255,0,0),3)
#             M = cv2.moments(cnt)

#             cx = int(M["m10"] / M["m00"])
#             cy = int(M["m01"] / M["m00"])

            

#             cv2.circle(real_img, (cx,cy),4,(255,0,0), -1)
#             centre_text = "( " + str(cx) + " , " + str(cy) + " )"
#             cv2.putText(real_img, centre_text , (cx-20, cy-20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,0),1)     
#             result=cv2.bitwise_and(real_img, real_img, mask=m_img)
            
#             cv2.imshow("real_img", real_img)
#             cv2.imshow("m_img", result)

    
#     print("Area: ", area, "\n")
#     print("(cx, cy): (", cx, " , ", cy, " )")
#     x=cx
#     y=cy
#     kinematics(x,y)
    

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

Distance = 20
while True:
  success,img=cap.read()
  img = cv2.flip(img, 1)
  img=cv2.resize(img,(640,480))
  HSV_img=cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
  # stak=np.hstack((img,HSV_img))
  h_min=cv2.getTrackbarPos('Hue Min','TrackBars')
  h_max=cv2.getTrackbarPos('Hue Max','TrackBars')
  s_min=cv2.getTrackbarPos('Sat Min','TrackBars')
  s_max=cv2.getTrackbarPos('Sat Max','TrackBars')
  v_min=cv2.getTrackbarPos('Val Min','TrackBars')
  v_max=cv2.getTrackbarPos('Val Max','TrackBars')
  # print(h_min,h_max,s_min,s_max,v_min,v_max)
  lower=np.array([h_min,s_min,v_min])
  upper=np.array([h_max,s_max,v_max])
  mask = cv2.inRange(HSV_img,lower,upper)
  
  obj_width_in_frame=obj_data(mask, img)
  
  if obj_width_in_frame != 0:
    Distance = Distance_finder(Focal_length_found, Known_width, obj_width_in_frame)
    cv2.putText(img, f"Distance: {round(Distance,2)} CM", (30, 35),cv2.FONT_HERSHEY_COMPLEX, 0.6, (255,0,0), 2)
 
  kinematics(Distance)
  print("Distance: ", Distance)
  cv2.imshow("FRAME",img)
  if cv2.waitKey(1) & 0xFF==ord('q'):
    break



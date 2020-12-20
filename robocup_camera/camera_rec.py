import cv2
from cv2 import data
import numpy as np
import math
from numpy.core.defchararray import greater
from numpy.core.numeric import NaN

from numpy.core.records import get_remaining_size

lower_green=np.array([100,100,70])
upper_green=np.array([120,225,220])

lower_yellow=np.array([10,100,125])
upper_yellow=np.array([22,235,180])

lower_ball=np.array([0,100,53])
upper_ball=np.array([0,100,100])

cap = cv2.VideoCapture(0)
while True:
    x_list_grren=[]
    y_list_green=[]
    x_list_yellow=[]
    y_list_yellow=[]
    green=[]
    yellow=[]
    ball=[]
    min_range_green=()
    min_range_yellow=()
    _,frame=cap.read("./9.bmp")
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask_green = cv2.inRange(hsv, lower_green, upper_green)
    mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)
    mask_ball=cv2.inRange(hsv, lower_ball, upper_ball)
    contours, hierarchy  = cv2.findContours(mask_ball, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    rects = []
    for contour in contours:
        approx = cv2.convexHull(contour)
        rect = cv2.boundingRect(approx)
        rects.append(np.array(rect))
    if len(rects) > 0:
        ball = max(rects, key=(lambda x: x[2] * x[3]))
        if ball[2]*ball[3]<=10:
            ball=0
    goal_green,trash_green=cv2.findContours(mask_green, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    goal_yellow,trash_yellow=cv2.findContours(mask_yellow, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    if len(goal_green)>0:
        green_pos=max(goal_green,key=cv2.contourArea)
        if cv2.contourArea(green_pos)>1000:
            buf_green=green_pos.flatten()
            for i, elem in enumerate(buf_green):
                if i%2==0:
                    x_list_grren.append(elem)
                else:
                    y_list_green.append(elem*(-1))
            green.append(x_list_grren)
            green.append(y_list_green)
        else:
            green=np.nan
            green=[[green,green]]
    else:
        green=np.nan
        green=[[green,green]]
    green=np.array(green).T.tolist()
    
    if len(goal_yellow)>0:
        yellow_pos=max(goal_yellow,key=cv2.contourArea)
        if cv2.contourArea(yellow_pos)>20:
            buf_yellow=yellow_pos.flatten()
            for i, elem in enumerate(buf_yellow):
                if i%2==0:
                    x_list_yellow.append(elem)
                else:
                    y_list_yellow.append(elem*(-1))
            yellow.append(x_list_yellow)
            yellow.append(y_list_yellow)
        else:
            yellow=np.nan
            yellow=[[yellow,yellow]]
    else:
        yellow=np.nan
        yellow=[[yellow,yellow]]
    yellow=np.array(yellow).T.tolist()
    
    if not math.isnan(green[0][0]):
        min_range_green=min(green,key=lambda i: (i[0]-160)**2+(120-i[1])**2)
    else:
        min_range_green=0
    if not math.isnan(yellow[0][0]):
        min_range_yellow=min(yellow,key=lambda i: (i[0]-160)**2+(120-i[1])**2)
    else:
        min_range_yellow=0
    print('\r green is'+str(min_range_green)+'yellow is'+str(min_range_yellow)+'ball is '+str(ball),end='')
    if cv2.waitKey(1) & 0xFF == 27:
        break
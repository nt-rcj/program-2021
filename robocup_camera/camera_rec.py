import cv2
from cv2 import data
import numpy as np
import math

lower_green=np.array([0,100,53])
upper_green=np.array([0,100,100])

lower_yellow=np.array([0,100,53])
upper_yellow=np.array([0,100,100])

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
    _, frame=cap.read()
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask_green = cv2.inRange(hsv, lower_green, upper_green)
    mask_yelllow = cv2.inRange(hsv, lower_yellow, upper_yellow)
    mask_ball=cv2.inRange(hsv, lower_ball, upper_ball)
    contours, hierarchy  = cv2.findContours(mask_ball, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    rects = []
    for contour in contours:
        approx = cv2.convexHull(contour)
        rect = cv2.boundingRect(approx)
        rects.append(np.array(rect))
    if len(rects) > 0:
        ball = max(rects, key=(lambda x: x[2] * x[3]))
    goal_green,trash_green=cv2.findContours(mask_green, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    goal_yellow,trash_yellow=cv2.findContours(mask_yelllow, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    green_pos=max(goal_green,key=cv2.contourArea())
    yellow_pos=max(goal_yellow,key=cv2.contourArea())
    buf_green=green_pos.flatten()
    buf_yellow=yellow_pos.flatten()
    for i, elem in enumerate(buf_green):
        if i%2==0:
            x_list_grren.append(elem)
        else:
            y_list_green.append(elem*(-1))
    green.append(x_list_grren)
    green.append(y_list_green)
    green=np.array(green).T.tolist()
    for i, elem in enumerate(buf_yellow):
        if i%2==0:
            x_list_yellow.append(elem)
        else:
            y_list_yellow.append(elem*(-1))
    yellow.append(x_list_yellow)
    yellow.append(y_list_yellow)
    yellow=np.array(green).T.tolist()
    min_range_green=min(green,key=lambda i: (i[0]-160)**2+(120-i[1])**2)
    min_range_yellow=min(yellow,key=lambda i: (i[0]-160)**2+(i[0]-160)**2)
    print('\r green %d yellow %f ball %g' %(min_range_green,min_range_yellow,ball),end='')
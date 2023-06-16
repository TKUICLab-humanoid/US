#!/usr/bin/env python
#coding=utf-8
import rospy
import numpy as np
import cv2
from Python_API import Sendmessage
import time
import timeit
import math


# def ball_cv2():
#     ball_x , ball_y , ball_r= 0 ,0 ,0
#     img = send.rawimg.copy()
#     gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
#     blur = cv2.GaussianBlur(gray,(5,5),0) 
#     # gray = cv2.bitwise_not(gray)
#     ret,thresh = cv2.threshold(blur,100,255,cv2.THRESH_BINARY) # 使用二值化閾值處理
#     circles = cv2.HoughCircles(thresh,cv2.HOUGH_GRADIENT,2,200,
#                             param1=33,param2=16,minRadius=3,maxRadius=15)        #thresh: 經過閾值處理的圖像
#                                                                                 # cv2.HOUGH_GRADIENT: Hough圓檢測的方法
#                                                                                 # 1: 圖像分辨率
#                                                                                 # 20: 圓之間的最小距離
#                                                                                 # param1: 檢測邊緣的閾值
#                                                                                 # param2: 檢測圓心的閾值
#                                                                                 # minRadius: 最小圓半徑
#                                                                                 # maxRadius: 最大圓半徑

#     if circles is not None:
#         circles = np.round(circles[0, :]).astype("int")
#         for (x, y, r) in circles:
#             ball_x = x
#             ball_y = y
#             ball_r = r

#     return ball_x,ball_y,ball_r

def BALL_img():
    color_XMax ,color_Xmin ,color_YMax ,color_Ymin ,color_X ,color_Y = 0, 0, 0, 0, 0 ,0 
    ball_yes = False
    ball_size = 0
    # print("ball size : ",max(send.color_mask_subject_size[5]))
    for i in range(send.color_mask_subject_cnts[color['Red']]):
        if send.color_mask_subject_size[5][i] <2500 and send.color_mask_subject_size[5][i] >350  and  abs(abs(send.color_mask_subject_XMax[color['Red']][i]-send.color_mask_subject_XMin[color['Red']][i])-abs(send.color_mask_subject_YMax[color['Red']][i]-send.color_mask_subject_YMin[color['Red']][i])) < 8:
            ball_yes = True
            color_XMax = send.color_mask_subject_XMax[color['Red']][i]
            color_Xmin = send.color_mask_subject_XMin[color['Red']][i]
            color_YMax = send.color_mask_subject_YMax[color['Red']][i]
            color_Ymin = send.color_mask_subject_YMin[color['Red']][i]
            color_X = send.color_mask_subject_X[color['Red']][i]
            color_Y = send.color_mask_subject_Y[color['Red']][i]
            ball_size = send.color_mask_subject_size[color['Red']][i]
            # rospy.loginfo(ball_size)
            # rospy.logdebug(abs(abs(send.color_mask_subject_XMax[color['Red']][i]-send.color_mask_subject_XMin[color['Red']][i])-abs(send.color_mask_subject_YMax[color['Red']][i]-send.color_mask_subject_YMin[color['Red']][i])))
    send.drawImageFunction(1,1,color_Xmin,color_XMax,color_Ymin,color_YMax,0,0,255)
    return color_X,color_Y,ball_yes,ball_size


def OBS_img():
    color_XMax ,color_Xmin ,color_YMax ,color_Ymin ,color_X ,color_Y ,= 0, 0, 0, 0, 0 ,0 
    obs_yes = False
    obs_size = 0
    # print("obs size : ",max(send.color_mask_subject_size[5]))
    
    if max(send.color_mask_subject_size[color['Blue']]) >2000 :
        obs_yes = True
        color_cnt = send.color_mask_subject_size[color['Blue']].index(max(send.color_mask_subject_size[color['Blue']]))
        color_XMax = send.color_mask_subject_XMax[color['Blue']][color_cnt]
        color_Xmin = send.color_mask_subject_XMin[color['Blue']][color_cnt]
        color_YMax = send.color_mask_subject_YMax[color['Blue']][color_cnt]
        color_Ymin = send.color_mask_subject_YMin[color['Blue']][color_cnt]
        color_X = send.color_mask_subject_X[color['Blue']][color_cnt]
        color_Y = send.color_mask_subject_Y[color['Blue']][color_cnt]
        obs_size = send.color_mask_subject_size[color['Blue']][color_cnt]
        rospy.loginfo(obs_size)
        rospy.logdebug(abs(abs(send.color_mask_subject_XMax[color['Blue']][color_cnt]-send.color_mask_subject_XMin[color['Blue']][color_cnt])-abs(send.color_mask_subject_YMax[color['Blue']][color_cnt]-send.color_mask_subject_YMin[color['Blue']][color_cnt])))
    send.drawImageFunction(2,1,color_Xmin,color_XMax,color_Ymin,color_YMax,0,255,255)
    return color_X,color_Y,obs_yes,obs_size


def ball_find(head_x,head_status):
    head_y_high = 1800
    head_y_low = 1500
    head_x_MAX = 2250
    head_x_min = 1750
    if head_status == 1:
        head_x = min(head_x,head_x_MAX)
        head_x += 10
        if head_x >= head_x_MAX:                #到達臨界值換狀態-->變成抬頭
            head_status = 2
        send.sendHeadMotor(1,head_x,100)
        send.sendHeadMotor(2,head_y_low,50)
        time.sleep(0.01)
        return head_x,head_status,head_y_low
    elif head_status == 2:
        head_x = max(head_x,head_x_min)
        head_x -= 10
        if head_x <= head_x_min:                #到達臨界值換狀態-->變成低頭
            head_status = 1
        send.sendHeadMotor(1,head_x,100)
        send.sendHeadMotor(2,head_y_high,50)
        time.sleep(0.01)
        return head_x,head_status,head_y_high

def obs_find(obs_color_X,obs_color_Y,obs_yes,obs_size):
    send.sendHeadMotor(1,2048,100)
    send.sendHeadMotor(2,1800,100)
    if  obs_color_Y >160:
        send.sendContinuousValue(1000,0,0,(obs_color_Y-160)//40,0)
    else:
        send.sendContinuousValue(1000,0,0,(-obs_color_Y)//40,0)


def catch_target(color_X,color_Y,head_x,head_y):
    if color_X < 150 :
        head_x += 10
    elif color_X > 170 :
        head_x -= 10
    if color_Y < 110:
        head_y += 10
    elif color_Y > 130:
        head_y -= 10
    print(color_X,color_Y)
    print(head_x,head_y)
    send.sendHeadMotor(1,head_x,100)
    send.sendHeadMotor(2,head_y,100)
    time.sleep(0.01)
    return head_x,head_y

def ball_go(head_x,send_theta):
    send_theta_min = -1
    send_theta_MAX = 2
    if head_x < 2000 :
        send_theta -= 1
        send_theta = max(send_theta,send_theta_min)
    elif head_x > 2100 :
        send_theta += 1
        send_theta = min(send_theta,send_theta_MAX)
    else:
        send_theta = 0

    print("send_theta",send_theta)
    send.sendContinuousValue(2000,0,0,send_theta,0)
    return send_theta

if __name__ == '__main__':
    try:
        send = Sendmessage()
        r = rospy.Rate(20)
        status = "find_ball"
        color = {   'Orange': 0,
                    'Yellow': 1,
                    'Blue':   2,
                    'Green':  3,
                    'Black':  4,
                    'Red':    5,
                    'White':  6 }
        first_in = True
        head_x = 1400
        head_y = 2048
        head_status = 1#狀態1是低頭2是抬頭
        send_theta = 0
        while not rospy.is_shutdown():#劃出起始十字
            # send.drawImageFunction(4,0,0,320,120,120,0,0,0)
            # send.drawImageFunction(5,0,160,160,0,240,0,0,0)
    
            if send.Web == True :#啟動電源與擺頭
                print("send_theta",send_theta)
                if first_in == True:
                    send.sendBodyAuto(2000,0,0,0,1,0)
                    first_in = False
                color_X,color_Y,ball_yes,ball_size = BALL_img()
                obs_color_X,obs_color_Y,obs_yes,obs_size = OBS_img()
                if ball_yes != True:
                    print("status : ",status)
                    head_x ,head_status,head_y = ball_find(head_x,head_status)
                    if obs_yes == True:
                        obs_find(obs_color_X,obs_color_Y,obs_yes,obs_size)
                else:
                    # send.sendBodyAuto(2000,0,0,0,1,0)
                    if head_y <= 1200:
                        send.sendBodyAuto(0,0,0,0,1,0)
                        time.sleep(1)
                        send.sendBodySector(111)
                        time.sleep(10)
                    status = "go to ball"
                    print("status : ",status)
                    head_x,head_y = catch_target(color_X,color_Y,head_x,head_y)
                    send_theta = ball_go(head_x,send_theta)


            if send.Web == False :
                # if first_in == False:
                #     # send.sendBodyAuto(2000,0,0,0,1,0)
                #     send.sendHeadMotor(1,2048,100)
                #     time.sleep(0.01)
                #     send.sendHeadMotor(2,2048,100)
                #     time.sleep(0.01)
                #     first_in = True
                print('aa')


            r.sleep()
    except rospy.ROSInterruptException:
        pass

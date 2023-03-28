#!/usr/bin/env python
#coding=utf-8
import rospy
import numpy as np
from Python_API import Sendmessage
import time
import timeit
import math

def ball_img():
    color_XMax ,color_Xmin ,color_YMax ,color_Ymin ,color_X ,color_Y ,color_cnt= 0, 0, 0, 0, 0 ,0 ,0
    ball_yes = False
    print("ball size : ",max(send.color_mask_subject_size[5]))
    if max(send.color_mask_subject_size[5]) >350:
        ball_yes = True
        color_cnt = send.color_mask_subject_size[color['Red']].index(max(send.color_mask_subject_size[color['Red']]))
        color_XMax = send.color_mask_subject_XMax[color['Red']][color_cnt]
        color_Xmin = send.color_mask_subject_XMin[color['Red']][color_cnt]
        color_YMax = send.color_mask_subject_YMax[color['Red']][color_cnt]
        color_Ymin = send.color_mask_subject_YMin[color['Red']][color_cnt]
        color_X = send.color_mask_subject_X[color['Red']][color_cnt]
        color_Y = send.color_mask_subject_Y[color['Red']][color_cnt]

    send.drawImageFunction(1,1,color_Xmin,color_XMax,color_Ymin,color_YMax,0,0,255)
    return color_X,color_Y,ball_yes


def ball_find(head_x,head_status):
    head_y_high = 2100
    head_y_low = 1500
    head_x_MAX = 2600
    head_x_min = 1400
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


def catch_target(color_X,color_Y,head_x,head_y):
    if color_X < 150 :
        head_x += 20
    elif color_X > 170 :
        head_x -= 20
    if color_Y < 110:
        head_y += 20
    elif color_Y > 130:
        head_y -= 20
    print(color_X,color_Y)
    print(head_x,head_y)
    send.sendHeadMotor(1,head_x,100)
    send.sendHeadMotor(2,head_y,100)
    time.sleep(0.01)
    return head_x,head_y

def ball_go(head_x,send_theta):
    send_theta_min = -3
    send_theta_MAX = 1
    if head_x < 2000 :
        send_theta -= 1
        send_theta = max(send_theta,send_theta_min)
    elif head_x > 2100 :
        send_theta += 1
        send_theta = min(send_theta,send_theta_MAX)
    else:
        send_theta = 0

    print("send_theta",send_theta)
    send.sendContinuousValue(100,0,0,send_theta,0)
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
                if first_in == True:
                    send.sendBodyAuto(2000,0,0,0,1,0)
                    first_in = False
                color_X,color_Y,ball_yes = ball_img()
                if ball_yes != True:
                    print("status : ",status)
                    head_x ,head_status,head_y = ball_find(head_x,head_status)
                else:
                    # send.sendBodyAuto(2000,0,0,0,1,0)
                    status = "go to ball"
                    print("status : ",status)
                    head_x,head_y = catch_target(color_X,color_Y,head_x,head_y)
                    send_theta = ball_go(head_x,send_theta)

            if send.Web == False :
                if first_in == False:
                    send.sendBodyAuto(2000,0,0,0,1,0)
                    send.sendHeadMotor(1,2048,100)
                    time.sleep(0.01)
                    send.sendHeadMotor(2,2048,100)
                    time.sleep(0.01)
                    first_in = True
                print('aa')


            r.sleep()
    except rospy.ROSInterruptException:
        pass

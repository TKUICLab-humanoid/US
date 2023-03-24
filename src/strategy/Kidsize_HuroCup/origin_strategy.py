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
    print(max(send.color_mask_subject_size[5]))
    if max(send.color_mask_subject_size[5]) >500:
        ball_yes = True
        color_cnt = send.color_mask_subject_size[color['Red']].index(max(send.color_mask_subject_size[color['Red']]))
        color_XMax = send.color_mask_subject_XMax[color['Red']][color_cnt]
        color_Xmin = send.color_mask_subject_XMin[color['Red']][color_cnt]
        color_YMax = send.color_mask_subject_YMax[color['Red']][color_cnt]
        color_Ymin = send.color_mask_subject_YMin[color['Red']][color_cnt]
        color_X = send.color_mask_subject_X[color['Red']][color_cnt]
        color_Y = send.color_mask_subject_Y[color['Red']][color_cnt]

    send.drawImageFunction(1,1,color_Xmin,color_XMax,color_Ymin,color_YMax,0,0,255)
    return color_X, color_Y, ball_yes

def ball_find():
    ball_yes = False
    head_x = 2048
    head_y = 1          #狀態1是低頭2是抬頭
    head_y_high = 2068
    head_y_low = 2028
    head_x_MAX = 2100
    head_x_min = 2000
    rospy.loginfo("ball_yes = %s",ball_yes)
    while ball_yes != True and send.Web == True:
        color_X, color_Y, ball_yes = ball_img()
        time.sleep(0.05)
        if head_y == 1 :
            head_x += 10 
        else :
            head_x -= 10 
        send.sendHeadMotor(1,head_x,100)    
        rospy.logdebug("head_x = %d",head_x)
        if  head_y == 1 and head_x >= head_x_MAX:
            send.sendHeadMotor(2,head_y_high,100)
            head_y = 2
            rospy.logdebug("head_y = %d",head_y)
        elif  head_y == 2 and head_x <= head_x_min:
            send.sendHeadMotor(2,head_y_low,100)
            head_y = 1
            rospy.logdebug("head_y = %d",head_y)
    if ball_yes ==True :
        head_x -= (color_X -160)* (65 / 320)
        if head_y == 1:
            send.sendHeadMotor(1,head_x,100)
            send.sendHeadMotor(2,head_y_low-(color_Y - 120)* (38 / 240) ,100)

    return head_x,head_y_low-(color_Y - 120)/2
        

def ball_go(head_x_instantly, head_y_instantly):
    ball_yes = True
    head_horizon = head_x_instantly
    head_vertical = head_y_instantly
    send.sendBodyAuto(2000,0,0,0,1,0)
    while ball_yes != False and send.Web == True:
        color_X, color_Y, ball_yes = ball_img()
        if abs(color_X - 160) > 2 or abs(color_Y - 120) > 2:
            x_differ =  color_X - 160 
            y_differ =  color_Y - 120 
            x_degree = x_differ * (65 / 320)
            y_degree = y_differ * (38 / 240)
            head_horizon -= int(x_degree * 4096 / 360 *0.15)
            head_vertical -= int(y_degree * 4096 / 360 *0.15)
            send.sendHeadMotor(1, head_horizon,50)
            send.sendHeadMotor(2, head_vertical,50)
            rospy.loginfo("head_horizon = %s",head_horizon)
            rospy.loginfo("head_vertical = %s",head_vertical)
        send.sendContinuousValue(2000,0,0,int((head_horizon-2048)/50),0)



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
        while not rospy.is_shutdown():#劃出起始十字
            # send.drawImageFunction(4,0,0,320,120,120,0,0,0)
            # send.drawImageFunction(5,0,160,160,0,240,0,0,0)
            
            if send.Web == True :#啟動電源與擺頭
                if status == "find_ball":
                    head_x_instantly, head_y_instantly = ball_find()
                    status = "go to ball"
                if status == "go to ball":
                    ball_go(head_x_instantly, head_y_instantly)
                    
            if send.Web == False :
                print('aa')


            r.sleep()
    except rospy.ROSInterruptException:
        pass
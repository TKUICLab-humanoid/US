#!/usr/bin/env python
#coding=utf-8
import rospy
import numpy as np
import cv2
from Python_API import Sendmessage
import time

HEAD_Y_HIGH = 1800
HEAD_Y_LOW = 1500
HEAD_X_MAX = 2250
HEAD_X_MIN = 1750

SEND_THETA_MIN = -4
SEND_THETA_MAX = 4

send = Sendmessage()

class United_Soccer:
    def __init__(self):
        self.obs = ObjectInfo('Blue', 'Obstacle')
        self.ball = ObjectInfo('Red', 'Ball')

        self.init()

    def ball_find(self):
        if self.head_status == "High":
            self.head_x = min(self.head_x, HEAD_X_MAX)
            self.head_x += 10
            if self.head_x >= HEAD_X_MAX:                #到達臨界值換狀態-->變成抬頭
                self.head_status = "Low"
            send.sendHeadMotor(1, self.head_x, 100)
            send.sendHeadMotor(2, HEAD_Y_LOW, 50)
            time.sleep(0.01)
        elif self.head_status == "Low":
            self.head_x = max(self.head_x, HEAD_X_MIN)
            self.head_x -= 10
            if self.head_x <= HEAD_X_MIN:                #到達臨界值換狀態-->變成低頭
                self.head_status = "High"
            send.sendHeadMotor(1, self.head_x, 100)
            send.sendHeadMotor(2, HEAD_Y_HIGH, 50)
            time.sleep(0.01)

    def obs_find(self):
        send.sendHeadMotor(1, 2048, 100)
        send.sendHeadMotor(2, 1800, 100)
        if self.obs.center.y >160:
            send.sendContinuousValue(1000, 0, 0, (self.obs.center.y - 160) // 40, 0)
        else:
            send.sendContinuousValue(1000, 0, 0, -(self.obs.center.y) // 40, 0)

    def catch_ball(self):
        if self.ball.center.x < 120:
            self.head_x += 10
        if self.ball.center.x > 140:
            self.head_x -= 10
        if self.ball.center.y < 110:
            self.head_y += 10
        if self.ball.center.y > 130:
            self.head_y -= 10
        rospy.logdebug(f'color_x = {self.ball.center.x} , color_y = {self.ball.center.y}')
        rospy.loginfo(f'head_x = {self.head_x} , head_y = {self.head_y}')
        send.sendHeadMotor(1, self.head_x, 100)
        send.sendHeadMotor(2, self.head_y, 100)
        time.sleep(0.01)

    def ball_go(self):
        if self.head_x < 2000:
            self.send_theta -= 1
            self.send_theta = max(self.send_theta, SEND_THETA_MIN)
        elif self.head_x > 2100:
            self.send_theta += 1
            self.send_theta = min(self.send_theta, SEND_THETA_MAX)
        else:
            self.send_theta = 0
        rospy.loginfo(f'send_theta = {self.send_theta}')
        send.sendContinuousValue(2000, 0, 0, self.send_theta, 0)

    def main(self):
        if send.is_start:#啟動電源與擺頭
            if self.first_in:
                send.sendBodyAuto(1000, 0, 0, 0, 1, 0)
                self.first_in = False

            self.ball.update()
            self.obs.update()
            rospy.loginfo(f'send_theta = {self.send_theta}')
            if not self.ball.get_target:
                if self.obs.get_target:
                    rospy.loginfo("avoid_obs")
                    self.obs_find()
                else:
                    rospy.loginfo("find_ball")
                    self.ball_find()
                    send.sendContinuousValue(1000, 0, 0, 0, 0)
            else:
                if self.head_y <= 1250:
                    send.sendBodyAuto(0, 0, 0, 0, 1, 0)
                    time.sleep(1)
                    send.sendBodySector(111)
                    time.sleep(4)
                    self.init()
                rospy.loginfo("goto_ball")
                self.catch_ball()
                self.ball_go()


        if not send.is_start:
            if not self.first_in:
                send.sendBodyAuto(2000, 0, 0, 0, 1, 0)
                send.sendHeadMotor(1, 2048, 100)
                time.sleep(0.01)
                send.sendHeadMotor(2, 2048, 100)
                time.sleep(0.01)
                self.first_in = True
            rospy.loginfo('aa')

    def init(self):
        self.head_x = 1400
        self.head_y = 2048
        self.send_theta = 0
        self.head_status = "High"
        self.first_in = True


class Coordinate:
    def __init__(self, x, y):
        self.x = x
        self.y = y

class ObjectInfo:
    color_dict = {'Orange': 0,
                  'Yellow': 1,
                  'Blue':   2,
                  'Green':  3,
                  'Black':  4,
                  'Red':    5,
                  'White':  6 }

    def __init__(self, color, object_type):
        self.color = self.color_dict[color]
        self.edge_max = Coordinate(0, 0)
        self.edge_min = Coordinate(0, 0)
        self.center = Coordinate(0, 0)
        self.get_target = False
        self.target_size = 0

        update_strategy = { 'Obstacle': self.get_obstacle_object,
                            'Ball': self.get_ball_object}
        self.find_object = update_strategy[object_type]

    def get_obstacle_object(self):
        if send.color_mask_subject_size[self.color] != []:
            max_object_size = max(send.color_mask_subject_size[self.color])
            max_object_idx = send.color_mask_subject_size[self.color].index(max_object_size)

            return max_object_idx if max_object_size > 15000 else None

    def get_ball_object(self):

        object_idx = None
        for i in range(send.color_mask_subject_cnts[self.color]):
            length_width_diff = abs(abs(send.color_mask_subject_XMax[self.color][i] - send.color_mask_subject_XMin[self.color][i]) - abs(send.color_mask_subject_YMax[self.color][i] - send.color_mask_subject_YMin[self.color][i]))
            if 100 < send.color_mask_subject_size[self.color][i] < 2500 and length_width_diff < 8:
                object_idx = i
        
        return object_idx

    def update(self):
        object_idx = self.find_object()

        if object_idx is not None:
            self.get_target = True
            self.edge_max.x = send.color_mask_subject_XMax[self.color][object_idx]
            self.edge_min.x = send.color_mask_subject_XMin[self.color][object_idx]
            self.edge_max.y = send.color_mask_subject_YMax[self.color][object_idx]
            self.edge_min.y = send.color_mask_subject_YMin[self.color][object_idx]
            self.center.x = send.color_mask_subject_X[self.color][object_idx]
            self.center.y = send.color_mask_subject_Y[self.color][object_idx]
            self.target_size = send.color_mask_subject_size[self.color][object_idx]

            rospy.loginfo(self.target_size)
            rospy.logdebug(abs(abs(self.edge_max.x - self.edge_min.x) - abs(self.edge_max.y - self.edge_min.y)))
            send.drawImageFunction(1, 1, self.edge_min.x, self.edge_max.x, self.edge_min.y, self.edge_max.y, 0, 0, 255)
        else:
            self.get_target = False


if __name__ == '__main__':

    try:
        strategy = United_Soccer()
        r = rospy.Rate(20)

        while not rospy.is_shutdown():
            strategy.main()
            r.sleep()
    except rospy.ROSInterruptException:
        pass
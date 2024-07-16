#!/usr/bin/env python
# coding=utf-8
from cmath import sqrt
from re import S, T
from traceback import print_tb
 
import numpy as np
import sys
# sys.path.append('/home/iclab/Desktop/adult_hurocup/src/strategy')
import rospy
from Python_API import Sendmessage

TEST                       = False
STAND_CORRECT              = False
#--影像中像素點對到中心對於頭部刻度的角度比例--#
X_RATIO                    = 64.5
Y_RATIO                    = 40
#------------------------------------------#
GOAL_IMU                   = [23,-23]
#
CORRECT                    = [-200,    0,      0]
#                            [大前進, 前進, 小前進, 原地, 小前進,   後退, 大後退]
FORWARD                    = [2500, 1000,   500,    0,  -500, -1000, -2000]
#                            [大左移,左移,小左移, 原地, 小右移,  右移,大右移]
TRANSLATION                = [2000, 1000,   500,    -600,  -2000, -2500, -2500]
#                            [大左旋,左旋, 原地,   右旋,大右旋]
THETA                      = [   7,    3,    0,    -3,    -7]
#                            [kick_center.x,kick_center.y,shot_center.x]
LEFT_POINT                 = [          130,          170,          195]
RIGHT_POINT                = [          170,          150,          80]
STRAIGHT_POINT_L           = [          150,          145,          140]
STRAIGHT_POINT_R           = [          170,          145,          180]
#----------#                       右腳           左腳
#                              左 ,  中,  右 |  左,  中,   右
FOOT                       = [105 , 124, 143, 160, 176, 196]
#
DODGE_OBS_ANGLE            = 1000
KICK_DEGREE_VERTICAL       = 1245
SHOT_DEGREE                = 1300
HEAD_ERROR_RANGE_X         = 15
HEAD_ERROR_RANGE_Y         = 35
ROTATE_ERROR               = 40 
#
DRAW_FUNCTION_FLAG         = True                  #影像繪圖開關
HEAD_HORIZONTAL            = 2048                  #頭水平
HEAD_VERTICAL              = 1350                  #頭垂直 
MAX_HEAD_HORIZONTAL        = 3072                  #頭水平最大
MIN_HEAD_HORIZONTAL        = 1024                  #頭水平最小
MAX_HEAD_VERTICAL          = 2048                  #頭垂直最大
MIN_HEAD_VERTICAL          = 1024                  #頭垂直最小

class Coordinate:
#儲存座標
    def __init__(self, x, y):
        self.x = x
        self.y = y

class ObjectInfo:
#物件的影件資訊
    color_dict = {  'Orange':  0,
                    'Yellow':  1,
                    'Blue'  :  2,
                    'Green' :  3,
                    'Black' :  4,
                    'Red'   :  5,
                    'White' :  6 }
    
    parameter  = {  'Orange':  2**0,
                    'Yellow':  2**1,
                    'Blue'  :  2**2,
                    'Green' :  2**3,
                    'Black' :  2**4,
                    'Red'   :  2**5,
                    'White' :  2**6 }
    def __init__(self, color, object_type,api):

        self.api              = api
        self.color            = self.color_dict[color]
        self.color_parameter  = self.parameter[color]
        self.edge_max         = Coordinate(0, 0)
        self.edge_min         = Coordinate(0, 0)
        self.center           = Coordinate(0, 0)
        self.get_target       = False
        self.target_size      = 0

        update_strategy = { 'OBS_left'     : self.get_left_obs_object,
                            'OBS_right'    : self.get_right_obs_object,
                            'Line'    : self.get_line_object,
                            'Ball'    : self.get_ball_object}
        self.find_object = update_strategy[object_type]

    def get_left_obs_object(self):
        object_idx = None
        for i in range(self.api.color_mask_subject_cnts[self.color]):
            if 2000 < self.api.color_mask_subject_size[self.color][i] and self.api.color_mask_subject_XMax[self.color][i] < 180:
                object_idx = i
        
        return object_idx

    def get_right_obs_object(self):
        object_idx = None
        for i in range(self.api.color_mask_subject_cnts[self.color]):
            if 2000 < self.api.color_mask_subject_size[self.color][i] and self.api.color_mask_subject_XMin[self.color][i] > 140:
                object_idx = i
        
        return object_idx

    def get_line_object(self):
        if self.api.color_mask_subject_size[self.color] != []:
            max_object_size = max(self.api.color_mask_subject_size[self.color])
            max_object_idx = self.api.color_mask_subject_size[self.color].index(max_object_size)
            return max_object_idx if max_object_size > 2000 else None

    def get_ball_object(self):
        object_idx = None
        for i in range(self.api.color_mask_subject_cnts[self.color]):
            length_width_diff = abs(abs(self.api.color_mask_subject_XMax[self.color][i] - self.api.color_mask_subject_XMin[self.color][i]) - abs(self.api.color_mask_subject_YMax[self.color][i] - self.api.color_mask_subject_YMin[self.color][i]))
            if 500 < self.api.color_mask_subject_size[self.color][i] and length_width_diff < 12:
                object_idx = i
        
        return object_idx

    def update(self):
        object_idx = self.find_object()

        if object_idx is not None:
            self.get_target  = True
            self.edge_max.x  = self.api.color_mask_subject_XMax[self.color][object_idx]
            self.edge_min.x  = self.api.color_mask_subject_XMin[self.color][object_idx]
            self.edge_max.y  = self.api.color_mask_subject_YMax[self.color][object_idx]
            self.edge_min.y  = self.api.color_mask_subject_YMin[self.color][object_idx]
            self.center.x    = self.api.color_mask_subject_X[self.color][object_idx]
            self.center.y    = self.api.color_mask_subject_Y[self.color][object_idx]
            self.target_size = self.api.color_mask_subject_size[self.color][object_idx]

            # rospy.loginfo(self.target_size)
            # rospy.loginfo(abs(abs(self.edge_max.x - self.edge_min.x) - abs(self.edge_max.y - self.edge_min.y)))
        else:
            self.edge_max.x  = 0
            self.edge_min.x  = 0
            self.edge_max.y  = 0
            self.edge_min.y  = 0
            self.center.x    = 0
            self.center.y    = 0
            self.target_size = 0
            self.get_target = False

class UnitedSoccer():
    def __init__(self,api):
        self.api        = api
        # self.pre_state  = 'stand'
        self.ball       = ObjectInfo( "Orange", 'Ball', self.api)
        self.obs_right  = ObjectInfo( "Blue"  , 'OBS_right', self.api)
        self.obs_left   = ObjectInfo( "Blue"  , 'OBS_left', self.api)  
        self.line       = ObjectInfo( "White" , 'Line', self.api)
        self.init()
        
    def init(self):
    #初始化
        self.object_center   = False
        self.walk_flag       = False
        self.state           = "find_ball"
        self.search          = 'right'
        #重置頭位置
        self.head_horizon    = HEAD_HORIZONTAL
        self.head_vertical   = HEAD_VERTICAL
        #步態
        self.forward         = 0
        self.translation     = 0
        self.theta           = 0 
        self.now_forward     = 0 
        self.now_translation = 0
        self.now_theta       = 0 
        self.imu_error       = 0
        self.search_count    = 0
        self.api.sendSensorReset(1,1,1)
        if TEST:
            self.drawImageFunction('start')
        else:
            self.drawImageFunction('init')
            self.control_head(1, self.head_horizon, 30) 
            self.control_head(2, self.head_vertical, 30)
        
    def drawImageFunction(self,state):
    #繪圖
        rospy.loginfo(f"now state:{self.state}")
        rospy.loginfo(f"水平刻度:{self.head_horizon},垂直刻度:{self.head_vertical}")
        rospy.logdebug(f"ball.target_size:{self.ball.target_size}")
        rospy.logdebug(f"ball.x:{self.ball.center.x},ball.y:{self.ball.center.y}")
        rospy.logdebug(f"now_imu:{self.api.imu_value_Yaw + self.imu_error}")
        if DRAW_FUNCTION_FLAG:
            if state == 'init':
                self.api.drawImageFunction(1, 0, 0, 0, 0, 0, 0, 255, 0)
                self.api.drawImageFunction(2, 0, 0, 0, 0, 0, 0, 255, 0)
                self.api.drawImageFunction(3, 1, 0, 0, 0, 0, 0, 127, 255)
                self.api.drawImageFunction(4, 1, 0, 0, 0, 0, 0, 127, 255)
                self.api.drawImageFunction(5, 1, 0, 0, 0, 0, 0, 127, 255)
                self.api.drawImageFunction(6, 1, 0, 0, 0, 0, 0, 127, 255)
                self.api.drawImageFunction(7, 1, 0, 0, 0, 0, 255, 128, 0)
                self.api.drawImageFunction(8, 1, 0, 0, 0, 0, 255, 13, 166)
                
            elif state == 'start':
                self.api.drawImageFunction(1, 0, 160, 160, 110, 130, 255, 255, 255)
                self.api.drawImageFunction(2, 0, 150, 170, 120, 120, 255, 255, 255)
                self.api.drawImageFunction(3, 0, self.ball.edge_min.x,\
                                                 self.ball.edge_min.x,\
                                                 self.ball.edge_min.y,\
                                                 self.ball.edge_max.y,\
                                                 0, 127, 255)
                self.api.drawImageFunction(4, 0, self.ball.edge_max.x,\
                                                 self.ball.edge_max.x,\
                                                 self.ball.edge_min.y,\
                                                 self.ball.edge_max.y,\
                                                 0, 127, 255)
                self.api.drawImageFunction(5, 0, self.ball.edge_min.x,\
                                                 self.ball.edge_max.x,\
                                                 self.ball.edge_min.y,\
                                                 self.ball.edge_min.y,\
                                                 0, 127, 255)
                self.api.drawImageFunction(6, 0, self.ball.edge_min.x,\
                                                 self.ball.edge_max.x,\
                                                 self.ball.edge_max.y,\
                                                 self.ball.edge_max.y,\
                                                 0, 127, 255)
                self.api.drawImageFunction(7, 1, self.obs_right.edge_min.x,\
                                                 self.obs_right.edge_max.x,\
                                                 self.obs_right.edge_min.y,\
                                                 self.obs_right.edge_max.y,\
                                                 255, 128, 0)
                self.api.drawImageFunction(8, 1, self.obs_left.edge_min.x,\
                                                 self.obs_left.edge_max.x,\
                                                 self.obs_left.edge_min.y,\
                                                 self.obs_left.edge_max.y,\
                                                 255, 13, 166)
                
    def object_update(self):
    #更新物件資訊
        self.ball.update()
        self.obs_right.update()
        self.obs_left.update()

    def control_head(self, ID, position, speed):
    #控制頭轉動
        if ID == 1:
        #水平
            if position > MAX_HEAD_HORIZONTAL:      #超過最大值
                self.head_horizon = MAX_HEAD_HORIZONTAL
            elif position < MIN_HEAD_HORIZONTAL:    #超過最小值
                self.head_horizon = MIN_HEAD_HORIZONTAL
            else:
                self.head_horizon = position

            self.api.sendHeadMotor(ID, self.head_horizon, speed)

        elif ID == 2:
        #垂直
            if position > MAX_HEAD_VERTICAL:      #超過最大值
                self.head_vertical = MAX_HEAD_VERTICAL
            elif position < MIN_HEAD_VERTICAL:    #超過最小值
                self.head_vertical = MIN_HEAD_VERTICAL
            else:
                self.head_vertical = position
            # self.head_vertical = self.degree_IMU_correct(self.head_vertical)
            self.api.sendHeadMotor(ID, self.head_vertical, speed)

    def control_walkinggait(self, forward, translation, theta, add_forward, add_translation, add_theta, reduce_forward, reduce_translation, reduce_theta):
    #控制步態參數
        if self.now_forward < forward:
            self.now_forward += add_forward
        elif self.now_forward > forward:
            self.now_forward -= reduce_forward
        else:
            self.now_forward = forward

        if self.now_translation < translation:
            self.now_translation += add_translation
        elif self.now_translation > translation:
            self.now_translation -= reduce_translation
        else:
            self.now_translation = translation

        if self.now_theta < theta:
            self.now_theta += add_theta
        elif self.now_theta > theta:
            self.now_theta -= reduce_theta
        else:
            self.now_theta = theta

        self.api.sendContinuousValue(self.now_forward, self.now_translation, 0, round(self.now_theta), 0)

    def search_object(self, right_max = 2048-600, left_max = 2048+600, up_max = 2048, down_max = 2048-300 , scale = 30):
    #找球 右->下->左->上
        if self.search == 'right':
            self.control_head(1, self.head_horizon, scale)
            self.head_horizon -= scale
            if self.head_horizon < right_max:
                self.head_horizon = right_max
                self.search = 'down'

        elif self.search == 'down':
            self.control_head(2, self.head_vertical, scale)
            self.head_vertical -= scale
            if self.head_vertical < down_max:
                self.head_vertical = down_max
                self.search = 'left'

        elif self.search == 'left':
            self.control_head(1, self.head_horizon, scale)
            self.head_horizon += scale
            if self.head_horizon > left_max:
                self.head_horizon = left_max
                self.search = 'up'

        elif self.search == 'up':
            self.control_head(2, self.head_vertical, scale)
            self.head_vertical += scale
            if self.head_vertical > up_max:
                self.head_vertical = up_max
                self.search = 'right'
           
    def trace_object(self, x_target, y_target,*,control_motor = 'double'):
    #追蹤目標物
        if x_target != 0 and y_target != 0:
            self.x_differ = x_target - 160
            self.y_differ = y_target - 120
            #--角度測量--#
            ##根據每個像素點和中心點的距離計算對應的角度(依機器人的高度去測量比例)
            self.x_degree = self.x_differ * (X_RATIO / 320)
            self.y_degree = self.y_differ * (Y_RATIO / 240)
            #------------#
            self.head_horizon  -= round(self.x_degree * 4096 / 360 * 0.15)
            self.head_vertical -= round(self.y_degree * 4096 / 360 * 0.15)
            if control_motor == 'doble' or 'horizon':
                self.control_head(1, self.head_horizon, 25)
            if control_motor == 'doble' or 'vertical':
                self.control_head(2, self.head_vertical, 25)
            if self.head_horizon - x_target < 5 and self.head_vertical - y_target <5:
                self.object_center = True
        else:
            rospy.logwarn(f"看不到球,重新找球")

    def body_trace_rotate(self,error):
    #修正機器人和目標的角度(轉頭用)
        rotate_error = self.head_horizon - HEAD_HORIZONTAL
        if rotate_error > error:
        #左轉修正
            self.control_walkinggait(FORWARD[3]+CORRECT[0], TRANSLATION[3]+CORRECT[1], THETA[1]+CORRECT[2], 500, 500, 1, 500,500, 1)
            rospy.logdebug("左轉")
            return False
        elif rotate_error < -error:
        #右轉修正
            self.control_walkinggait(FORWARD[3]+CORRECT[0], TRANSLATION[3]+CORRECT[1], THETA[3]+CORRECT[2], 500, 500, 1, 500, 500, 1)
            rospy.logdebug("右轉")
            return False
        return True

    def body_trace_straight(self,goal_degree,error):
    #修正機器人與目標的直線距離(轉頭用)
        distance_error = self.head_vertical - goal_degree
        if distance_error > error:
            return FORWARD[0] + CORRECT[0]
        elif distance_error < -error:
            return FORWARD[4] + CORRECT[0]
        else:
            return FORWARD[2] + CORRECT[0]

    def body_trace_translation(self,goal_degree,error):
    #修正機器人與目標的平移距離(轉頭用)
        translation_error = self.head_horizon - goal_degree
        # print(translation_error,goal_degree)
        if translation_error > 250:
            return TRANSLATION[0] + CORRECT[1]
        elif translation_error < -250:
            return TRANSLATION[4] + CORRECT[1]
        elif translation_error > error:
            return TRANSLATION[1] + CORRECT[1]
        elif translation_error < -error:
            return TRANSLATION[3] + CORRECT[1]
        else:
            return TRANSLATION[2] + CORRECT[1]

    def imu_correct(self,fix,error_range):
    #讓身體imu回正
        yaw_error = self.api.imu_value_Yaw - fix 
        print(yaw_error)
        if abs(yaw_error) >=error_range + 10:
            if yaw_error > 0:#右修
                rospy.logdebug("大右轉")
                return THETA[4] + CORRECT[2]
            elif yaw_error < 0:#左修
                rospy.logdebug("大左轉")
                return THETA[0] + CORRECT[2]
        elif abs(yaw_error) >=error_range:
            if yaw_error > 0:#右修
                rospy.logdebug("右轉")
                return THETA[3] + CORRECT[2]
            elif yaw_error < 0:#左修
                rospy.logdebug("左轉")
                return THETA[1] + CORRECT[2]
        return THETA[2] + CORRECT[2]

    def walk_change(self,walk_flag):
        rospy.sleep(0.5)
        if walk_flag:
        #停下
            self.walk_flag = False
            self.api.sendBodyAuto(0, 0, 0, 0, 1, 0)
            rospy.sleep(2.5)
            self.api.sendBodySector(29)
            rospy.sleep(1)
        else:
        #開始走路
            self.walk_flag = True
            self.api.sendBodyAuto(FORWARD[2]+CORRECT[0], TRANSLATION[3]+CORRECT[1], 0, THETA[3]+CORRECT[2], 1, 0)

    def get_up(self,imu):
        if imu > 15:
        #向前倒
            self.api.sendHeadMotor(1, 2048, 0)
            self.api.sendHeadMotor(2, 2420, 0)
            self.api.sendBodyAuto(0, 0, 0, 0, 1, 0)
            rospy.sleep(1.5)
            self.api.sendBodySector(29)
            rospy.sleep(1)
            self.api.sendBodySector(1212)
            rospy.sleep(19)
            self.api.sendBodySector(29)
            rospy.sleep(0.01)
            self.api.sendBodySector(1)
            rospy.sleep(1)
            self.init()
        elif imu < -15:
        #向後倒
            self.api.sendHeadMotor(1, 2048, 0)
            self.api.sendHeadMotor(2, 2420, 0)
            self.api.sendBodyAuto(0, 0, 0, 0, 1, 0)
            rospy.sleep(1.5)
            self.api.sendBodySector(29)
            rospy.sleep(1)
            self.api.sendBodySector(1211)
            rospy.sleep(10)
            self.api.sendBodySector(29)
            rospy.sleep(0.01)
            self.api.sendBodySector(1)
            rospy.sleep(1)
            self.init()
    
    def motion(self,state):
        if state == "find_ball":
        #找球
            if not self.ball.get_target:
                self.search_object(#right_max = 2048-600,\
                                   #left_max = 2048+600,\
                                   #up_max = 2048,\
                                   down_max = 2048-800,\
                                   scale = 50)
            else:
                self.state = "trace_ball"

        elif state == "trace_ball":
        #走到球前
            self.trace_object(self.ball.center.x,self.ball.center.y)
            if self.body_trace_rotate(60):
                self.api.sendContinuousValue(FORWARD[0]+CORRECT[0], TRANSLATION[3]+CORRECT[1], 0, THETA[2]+CORRECT[2], 0)
                if self.head_vertical < SHOT_DEGREE:
                    self.state = "find_obs"
                    self.object_center = False
                    self.walk_change(self.walk_flag)

        elif state == "find_obs":
            print(self.object_center)
            print(self.search_count)
            if self.obs_right.get_target or self.obs_left.get_target or self.search_count > 200:
            #找到OBS 或 尋找3次沒找到
                if self.obs_right.get_target:
                #右邊有OBS
                    self.trace_object(self.obs_right.center.x,self.obs_right.center.y)
                elif self.obs_left.get_target:
                #左邊有OBS
                    self.trace_object(self.obs_left.center.x,self.obs_left.center.y)

                if self.search_count > 2 or self.object_center:
                #準備踢球
                    self.state = "shot_ball"
                    self.object_center = False
                    rospy.sleep(10)

            else:
            #找OBS
                self.search_object(right_max = 2048-600-self.search_count,\
                                   left_max = 2048+600+self.search_count,\
                                   up_max = 2048,\
                                   down_max = 2048-800,\
                                   scale = 50)
                self.search_count += 1

        elif state == "shot_ball":
            pass
    # def imu_error_reset(self):
    #     self.imu_error = self.imu_error + self.api.imu_value_Yaw 
    #     self.api.sendSensorReset(1,1,1)

    def main(self): 
        if self.api.is_start:
            self.object_update()
            self.drawImageFunction('start')
            if not self.walk_flag and self.state != "find_obs":
                self.walk_change(self.walk_flag)
            self.motion(self.state)
        else:
            rospy.logdebug('strategy close')
            if TEST:
                self.object_update()
            if self.walk_flag:
                self.walk_change(self.walk_flag)
            self.init()
        

if __name__ == '__main__':
    try:
        rospy.init_node('US_strategy', anonymous=True, log_level=rospy.DEBUG)   #初始化node
        send = Sendmessage()
        us = UnitedSoccer(send)
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            us.main()
            r.sleep()
    except rospy.ROSInterruptException:
        pass
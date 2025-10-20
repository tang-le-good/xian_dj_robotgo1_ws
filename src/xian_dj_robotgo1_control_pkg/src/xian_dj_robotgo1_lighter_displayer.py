#!/usr/bin/env python3
#!coding=utf-8
import rospy
import signal
import sys
import numpy as np
import time, datetime, os, json, logging
     
class XianDjRobotgo1LighterDisplayer:
    def __init__(self):
        self.counter = 0
        
        # GPIO全局编号
        self.GPIO_green_lighter = 495  # 引脚13, 控制绿灯
        self.GPIO_red_lighter = 496  # 引脚15, 控制红灯
        self.GPIO_yellow_lighter = 83  # 引脚29, 控制黄灯
        
        self.xian_dj_robotgo1_green_ligher_cmd = 0
        self.xian_dj_robotgo1_red_ligher_cmd = 0
        self.xian_dj_robotgo1_yellow_ligher_cmd = 0
        self.xian_dj_robotgo1_displayer_cmd

        #引脚使能，并设置为输出模式
        self.gpio_export(self.GPIO_green_lighter)
        self.gpio_set_direction(self.GPIO_green_lighter, "out")
    
        self.gpio_export(self.GPIO_red_lighter)
        self.gpio_set_direction(self.GPIO_red_lighter, "out")

        self.gpio_export(self.GPIO_yellow_lighter)
        self.gpio_set_direction(self.GPIO_yellow_lighter, "out")

    def xian_heat_beat_callback(self, event):
        rospy.set_param("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_lighter_displayer_heart_beat", self.counter)
        xian_dj_robotgo1_lighter_displayer_heart_beat = rospy.get_param("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_lighter_displayer_heart_beat")
        if self.counter>1000:
            self.counter = 0
        self.counter += 1
        print("xian_dj_robotgo1_lighter_displayer_heart_beat:", xian_dj_robotgo1_lighter_displayer_heart_beat)
    
    
    def xian_dj_tobotgo1_green_lighter_func(self, event):
        self.xian_dj_robotgo1_green_ligher_cmd = rospy.get_param("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_green_ligher_cmd")
        self.gpio_set_value(self.GPIO_green_lighter, self.xian_dj_robotgo1_green_ligher_cmd)
        pass

    def xian_dj_tobotgo1_red_lighter_func(self, event):
        self.xian_dj_robotgo1_red_ligher_cmd = rospy.get_param("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_red_ligher_cmd")
        self.gpio_set_value(self.GPIO_red_lighter, self.xian_dj_robotgo1_red_ligher_cmd)
        pass

    def xian_dj_tobotgo1_yellow_lighter_func(self, event):
        self.xian_dj_robotgo1_yellow_ligher_cmd = rospy.get_param("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_yellow_ligher_cmd")
        self.gpio_set_value(self.GPIO_yellow_lighter, self.xian_dj_robotgo1_yellow_ligher_cmd)
        pass

    def xian_dj_tobotgo1_displayer_func(self, event):
        self.xian_dj_robotgo1_displayer_cmd = rospy.get_param("/xian_dj_robotgo1_params_server/xian_dj_robotgo1_displayer_cmd")
        # 把该变量发送给数显模块
        pass

    def gpio_export(self,pin):# 使能引脚
        export_path = f"/sys/class/gpio/gpio{pin}"
        if not os.path.exists(export_path):
            with open("/sys/class/gpio/export", "w") as f:
                f.write(str(pin))
            time.sleep(0.1)  # 等待系统创建目录
    
    def gpio_unexport(self, pin): # 复位引脚
        export_path = f"/sys/class/gpio/gpio{pin}"
        if os.path.exists(export_path):
            with open("/sys/class/gpio/unexport", "w") as f:
                f.write(str(pin))
    
    def gpio_set_direction(self,pin, direction):# 设置引脚为输出模式
        with open(f"/sys/class/gpio/gpio{pin}/direction", "w") as f:
            f.write(direction)

    def gpio_set_value(self,pin, value):# 设置引脚的高低电平
        with open(f"/sys/class/gpio/gpio{pin}/value", "w") as f:
            f.write(str(value))



if __name__ == '__main__':
    try:
        tt = XianDjRobotgo1LighterDisplayer()
        rospy.init_node('xian_dj_robotgo1_lighter_displayer', anonymous=True)  # 初始化ROS节点
        rospy.Timer(rospy.Duration(1), tt.xian_heat_beat_callback, oneshot=False) # 心跳线程
        rospy.Timer(rospy.Duration(0.02), tt.xian_dj_tobotgo1_green_lighter_func, oneshot=False) # 绿灯线程
        rospy.Timer(rospy.Duration(0.02), tt.xian_dj_tobotgo1_red_lighter_func, oneshot=False) # 红灯线程
        rospy.Timer(rospy.Duration(0.02), tt.xian_dj_tobotgo1_yellow_lighter_func, oneshot=False) # 黄灯线程
        rospy.Timer(rospy.Duration(0.02), tt.xian_dj_tobotgo1_displayer_func, oneshot=False) # 数显模块线程
        rospy.spin()  # 添加这行确保节点持续运行

    except rospy.ROSInterruptException:
        pass
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import serial
import time
from std_msgs.msg import String

# 定义串口参数
#serialPort = "/dev/ttyUSB2"
baudRate = 9600

# 初始化串口
ser = serial.Serial(port= baudrate=baudRate, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE)

# 定义机器人射击类
class AbotShoot:
    def __init__(self):
        # 给节点命名
        rospy.init_node('abot_shoot', anonymous=False)
        
        # 订阅 /shoot 话题
        rospy.Subscriber('/shoot', String, self.shoot_continue)
        
        rospy.loginfo("Shoot to ar_tag")
        
    # 处理射击命令的回调函数
    def shoot_continue(self, msg):
        # 发送射击指令
        ser.write(b'\x55\x01\x12\x00\x00\x00\x01\x69')
        print(0)
        time.sleep(0.05)
        ser.write(b'\x55\x01\x11\x00\x00\x00\x01\x68')

if __name__ == '__main__':
    try:
        # 实例化 AbotShoot 类并运行节点
        abot_shoot = AbotShoot()
        rospy.spin()
    except Exception as e:
        # 打印错误信息
        rospy.logerr("Exception occurred: %s", str(e))

        

        


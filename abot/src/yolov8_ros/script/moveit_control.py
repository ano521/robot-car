#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from sensor_msgs.msg import JointState
import operator
import sys
#sys.path.append('/home/')
from WCXC_UartServer import  WCXC_UART
#myUart = WCXC_UartServer.WCXC_UART(115200)
class SubscriberNode:
    
    def __init__(self, name):
        self.last_pos = [0.0, 0.0, 0.0, 0.0, 0.0]  # 记录上一个关节位置
        self.cur_pos = [0.0, 0.0, 0.0, 0.0, 0.0]   # 记录当前关节位置
        self.myUart = WCXC_UART(115200)  # 初始化串口
        rospy.init_node(name, anonymous=True)  # ROS1节点初始化
        self.sub = rospy.Subscriber("/joint_states", JointState, self.jointstate_callback)  # 创建订阅者对象
        rospy.loginfo('wcxc_arm_ready')

    def jointstate_callback(self, msg):
        self.joint_position = list(msg.position)
        self.cur_pos = self.joint_position[:]
        
        if operator.eq(self.cur_pos, self.last_pos) is False:
            self.last_pos = self.cur_pos[:]    
            self.data_to_uart()

    def data_to_uart(self):
        servo_pwm0 = self.last_pos[0] * 57.3 * 2000 / 270
        servo_pwm1 = self.last_pos[1] * 57.3 * 2000 / 270
        servo_pwm2 = self.last_pos[2] * 57.3 * 2000 / 270
        servo_pwm3 = self.last_pos[3] * 57.3 * 2000 / 270
        servo_pwm4 = self.last_pos[4] * 57.3 * 2000 / 270
        servo_pwm5 = self.last_pos[5] * 57.3 * 2000 / 270

        servo_pwm0 = int(servo_pwm0)
        servo_pwm1 = int(servo_pwm1)
        servo_pwm2 = int(servo_pwm2)
        servo_pwm3 = int(servo_pwm3)
        servo_pwm4 = int(servo_pwm4)
        servo_pwm5 = int(servo_pwm5)

        textSrt = '#000P{0:0>4d}T{6:0>4d}!#001P{1:0>4d}T{6:0>4d}!#002P{2:0>4d}T{6:0>4d}!#003P{3:0>4d}T{6:0>4d}!#004P{4:0>4d}T{4:0>4d}!#005P{5:0>4d}T{6:0>4d}!'.format(
            1500 + servo_pwm0, 1500 + servo_pwm1, 1500 - servo_pwm2, 1500 + servo_pwm3, 1500 + servo_pwm4, 1500+servo_pwm5, 1000)
        rospy.loginfo('send_str:{}'.format(textSrt))
        self.myUart.uart_send_str(textSrt)

def main():
    node = SubscriberNode("wcxc_arm_control")
    rospy.spin()  # 循环等待ROS1退出

if __name__ == '__main__':
    main()


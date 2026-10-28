#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import serial
import time
from std_msgs.msg import String

# 定义机器人射击类
class AbotShoot:
    def __init__(self):
        # 给节点命名
        rospy.init_node('abot_shoot', anonymous=False)

        # Keep the actuator disabled unless the launch caller explicitly names
        # the serial device.  This avoids commanding an unknown USB device.
        self.serial_port = rospy.get_param('~serial_port', '')
        self.baud_rate = rospy.get_param('~baud_rate', 9600)
        self.ser = None
        if self.serial_port:
            try:
                self.ser = serial.Serial(
                    port=self.serial_port,
                    baudrate=self.baud_rate,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    timeout=1.0,
                )
            except serial.SerialException as exc:
                rospy.logerr('Cannot open shoot serial port %s: %s', self.serial_port, exc)
        else:
            rospy.logwarn('Shoot actuator is disabled: set ~serial_port to enable it.')

        # 订阅 /shoot 话题
        rospy.Subscriber('/shoot', String, self.shoot_continue)
        
        rospy.loginfo("Shoot to ar_tag")
        
    # 处理射击命令的回调函数
    def shoot_continue(self, msg):
        if self.ser is None:
            rospy.logerr('Ignoring /shoot command because no actuator serial port is open.')
            return

        # 发送射击指令
        self.ser.write(b'\x55\x01\x12\x00\x00\x00\x01\x69')
        time.sleep(0.05)
        self.ser.write(b'\x55\x01\x11\x00\x00\x00\x01\x68')

if __name__ == '__main__':
    try:
        # 实例化 AbotShoot 类并运行节点
        abot_shoot = AbotShoot()
        rospy.spin()
    except Exception as e:
        # 打印错误信息
        rospy.logerr("Exception occurred: %s", str(e))

        

        


#!/usr/bin/env python
import os
import sys
#sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import time
import threading
from math import *
import rospy
from dynamic_reconfigure.server import Server
from yolov8_ros.cfg import KeyboardGraspConfig

import WCXC_UartServer
import z_kinematics as kms

# 全局变量
carry_step = 0
Running = True
run_flag = True
servo_yuntai = 1500
servo_zhuazi = 1500
index = 0
servo_yuntai_bias = 0
stack_count = 0
kms_x = 0
kms_y = 0
kms_z = 10
move_time = 2000
last_trigger_action = False
last_reset_position = False
last_gripper_open = False
last_gripper_close = False
debug_mode = False

# 动态参数配置回调函数
def dynamic_reconfigure_callback(config, level):
    global kms_x, kms_y, kms_z, servo_yuntai, servo_zhuazi, move_time
    global last_trigger_action, last_reset_position, last_gripper_open, last_gripper_close, debug_mode
    
    # 更新位置参数
    kms_x = config.kms_x
    kms_y = config.kms_y
    kms_z = config.kms_z
    
    # 更新舵机参数
    servo_yuntai = config.servo_yuntai
    servo_zhuazi = config.servo_zhuazi
    
    # 更新运动时间
    move_time = config.move_time
    
    # 更新调试模式
    debug_mode = config.debug_mode
    
    # 检测触发器状态变化
    if config.trigger_action and not last_trigger_action:
        rospy.loginfo("触发抓取动作！位置: x={}, y={}, z={}".format(kms_x, kms_y, kms_z))
        eEvent.set()
    last_trigger_action = config.trigger_action
    
    if config.reset_position and not last_reset_position:
        rospy.loginfo("重置到初始位置")
        reset_to_initial_position()
    last_reset_position = config.reset_position
    
    if config.gripper_open and not last_gripper_open:
        rospy.loginfo("张开爪子")
        myUart.uart_send_str("#005P1200T1000!")
    last_gripper_open = config.gripper_open
    
    if config.gripper_close and not last_gripper_close:
        rospy.loginfo("闭合爪子")
        myUart.uart_send_str("#005P1700T1000!")
    last_gripper_close = config.gripper_close
    
    if debug_mode:
        rospy.loginfo("参数更新: x={}, y={}, z={}, 云台={}, 爪子={}, 时间={}ms".format(
            kms_x, kms_y, kms_z, servo_yuntai, servo_zhuazi, move_time))
    
    return config

# 重置到初始位置函数
def reset_to_initial_position():
    global myUart
    myUart.uart_send_str('#000P{0:0>4d}T3000!#001P1500T3000!#002P1800T3000!#003P0700T3000!#004P1500T3000!#005P1500T3000!'.format(int(1500 + 2000 * 0 / 270)))


# 夹取木块
def stcarry_wood():
    global carry_step, Running, run_flag, servo_yuntai, servo_zhuazi, index, servo_yuntai_bias, stack_count, kms_x, kms_y
    while 1:
        eEvent.wait()
        print('ok')
        if carry_step == 0:
            myUart.uart_send_str('#000P{0:0>4d}T1000!#001P1500T1000!#002P1800T1000!#003P0700T1000!#004P1500T1000!#005P1500T1000!'.format(int(1500 + 2000 * 0 / 270)))
            time.sleep(2)
            myUart.uart_send_str("#005P1200T0500!")
            time.sleep(1)
            carry_step = 1
        elif carry_step == 1:
            Str = "#000P{0:0>4d}T{0:0>4d}!#004P{1:0>4d}T{2:0>4d}!#005P1200T{2:0>4d}!" .format(servo_yuntai, servo_zhuazi, 1000)
            myUart.uart_send_str(Str)
            time.sleep(1)
            carry_step = 2

        # 机械臂运行到目标位置
        elif carry_step == 2:
            kinematics_move(kms_x, kms_y + 10, kms_z, move_time)
            time.sleep(move_time / 1000.0 + 1)
        #     carry_step = 3
        # # 闭合爪子
        # elif carry_step == 3:
        #     myUart.uart_send_str("#005P1700T1000!")
        #     time.sleep(2)
        #     carry_step = 4
        # # 抬起机械臂
        # elif carry_step == 4:
        #     kinematics_move(kms_x, kms_y + 10, 150, 1000)
        #     time.sleep(1)
        #     carry_step = 5
        # # 旋转机械臂
        # elif carry_step == 5:
        #     print('index1', index)
        #     myUart.uart_send_str('#000P2150T1000!#004P1500T1000!')
        #     time.sleep(2)
        #     carry_step = 6
        # # 放置木块
        # elif carry_step == 6:
        #     kinematics_move(-190 + stack_count * 2, 0, 10 + stack_count * 30, 1000)
        #     time.sleep(2)
        #     carry_step = 7
        # # 抬起机械臂
        # elif carry_step == 7:
        #     myUart.uart_send_str('#005P1200T1000!')
        #     time.sleep(2)
        #     carry_step = 8
        # # 回到初始位置
        # elif carry_step == 8:
        #     myUart.uart_send_str('{#001P2000T2000!#002P2200T2000!#003P0700T2000!#004P1500T2000!#005P1700T2000!}')
        #     time.sleep(2)
        #     carry_step = 9
        # elif carry_step == 9:
        #     stack_count += 1
        #     if stack_count == 4:
        #         stack_count = 0
        #     myUart.uart_send_str('#000P{0:0>4d}T{1:0>4d}!#001P1500T2000!#002P1800T2000!#003P0700T2000!#004P1500T2000!'.format(int(1500 - 2000 * (servo_yuntai_bias / 270)), 1000))
        #     time.sleep(2)
            Running = True
            carry_step = 0
            run_flag = True
            eEvent.clear()

# 逆运动学算法
def kinematics_move(x, y, z, mytime):
    global servo_angle, servo_pwm
    if y < 0:
        return 0
    # 寻找最佳角度
    flag = 0
    my_min = 0
    for i in range(-135, 0):
        if 0 == kms.kinematics_analysis(x, y, z, i):
            if (i < my_min):
                my_min = i
            flag = 1

    # 用3号舵机与水平最大的夹角作为最佳值
    if (flag):
        kms.kinematics_analysis(x, y, z, my_min);
        testStr = '{'
        for j in range(0, 4):
            # set_servo(j, servo_pwm[j], time);
            # print(servo_pwm[j])
            testStr += "#%03dP%04dT%04d!" % (j, kms.servo_pwm[j], mytime)
        testStr += '}'
        print(testStr)
        myUart.uart_send_str(testStr)
        return 1

    return 0

eEvent = threading.Event()
th1 = threading.Thread(target=stcarry_wood)
th1.setDaemon(True)
th1.start()

if __name__ == '__main__':
    # 初始化ROS节点
    rospy.init_node('keyboard_grasp_node', anonymous=True)
    rospy.loginfo("机械臂键盘控制节点启动")
    
    # 初始化串口和运动学
    myUart = WCXC_UartServer.WCXC_UART(115200)  # 设置串口
    kms.setup_kinematics(100, 105, 75, 185)
    
    # 初始化机械臂位置
    myUart.uart_send_str('#000P{0:0>4d}T1000!#001P1500T1000!#002P1800T1000!#003P0700T1000!#004P1500T1000!#005P1500T1000!'.format(int(1500 + 2000 * 0 / 270)))
    
    # 初始化动态配置服务器
    srv = Server(KeyboardGraspConfig, dynamic_reconfigure_callback)
    rospy.loginfo("动态参数配置服务器已启动")
    rospy.loginfo("请使用 'rosrun rqt_reconfigure rqt_reconfigure' 打开控制面板")
    
    # 运行ROS节点
    rospy.spin()

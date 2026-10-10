#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import time
import WCXC_UartServer
import z_kinematics as kms

myUart = WCXC_UartServer.WCXC_UART(115200)    #设置串口 


systick_ms_data = 0
get_id = 0

'''
设置四个关节的长度
单位0.1mm
l0  底盘到第二个舵机中心轴的距离
l1  第二个舵机到第三 个舵机的距离
l2  第三个舵机到第四 个舵机的距离
l3  第四个舵机到机械臂(闭合后)最高点的距离

'''
kms.setup_kinematics(110,105,75,185)#初始化

myUart.uart_send_str("#255P1500T1000!")
time.sleep(2)

def get_pwm_value():
    global systick_ms_data,get_id
    if(int((time.time() * 1000))- systick_ms_data > 100):
        systick_ms_data = int((time.time() * 1000))
        if get_id == 0:
            myUart.uart_send_str("#000PRAD!")
            if myUart.uart_get_ok:
                pwm_value0 = myUart.uart_receive_buf
                print(pwm_value0)
                myUart.uart_receive_buf = ''
                myUart.uart_get_ok = 0
                get_id = 1
        elif get_id == 1:
            myUart.uart_send_str("#001PRAD!")
            if myUart.uart_get_ok:
                pwm_value1 = myUart.uart_receive_buf
                print(pwm_value1)
                myUart.uart_receive_buf = ''
                myUart.uart_get_ok = 0
                get_id = 2
        elif get_id == 2:
            myUart.uart_send_str("#002PRAD!")
            if myUart.uart_get_ok:
                pwm_value2 = myUart.uart_receive_buf
                print(pwm_value2)
                myUart.uart_receive_buf = ''
                myUart.uart_get_ok = 0
                get_id = 3
        elif get_id == 3:
            myUart.uart_send_str("#003PRAD!")
            if myUart.uart_get_ok:
                pwm_value3 = myUart.uart_receive_buf
                print(pwm_value3)
                myUart.uart_receive_buf = ''
                myUart.uart_get_ok = 0
                get_id = 4
        elif get_id == 4:
            myUart.uart_send_str("#004PRAD!")
            if myUart.uart_get_ok:
                pwm_value4 = myUart.uart_receive_buf
                print(pwm_value4)
                myUart.uart_receive_buf = ''
                myUart.uart_get_ok = 0
                get_id = 5
        elif get_id == 5:
            myUart.uart_send_str("#005PRAD!")
            if myUart.uart_get_ok:
                pwm_value5 = myUart.uart_receive_buf
                print(pwm_value5)
                myUart.uart_receive_buf = ''
                myUart.uart_get_ok = 0
                get_id = 0

while 1:
    get_pwm_value()

'''
输入参数:x,y,z,time

以机械臂的视角来看：
左右为x轴,右为正值，单位毫米;
前后为y轴,前为正值,y值不能小于0,单位毫米;
z为高度,单位毫米;
time为机械臂运行到目标位置的时间,单位毫秒。

若输出为{#000P0000T1000!#001P0000T1000!#002P0000T1000!#003P0000T1000!}。说明机械臂无法达到目标位置。
'''

#kms.kinematics_move(0,100,150,1000)#运用
#time.sleep(2)
#kms.kinematics_move(0,200,200,1000)
#time.sleep(2)
#kms.kinematics_move(0,300,50,2000)
#time.sleep(3)
#myUart.uart_send_str("#255P1500T2000!")


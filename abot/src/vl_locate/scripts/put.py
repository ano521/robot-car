#!/usr/bin/env python3
'''
Copyright (c) [Zachary]
本代码受版权法保护，未经授权禁止任何形式的复制、分发、修改等使用行为。
Author:Zachary
company:WCXC
'''
import os
import sys
#sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import time
import threading
from math import *
from std_msgs.msg import String
import WCXC_UartServer
import z_kinematics as kms
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped

# 画面高度
img_w = 640
img_h = 480
theta6=0
#搬运过程标志
systick_ms_bak = 0
carry_step = 0
next_time = 50
zhuan_flag = True
systick_ms_bak_zhuan = 0
servo_yuntai_bias = 0
run_flag = True
testStr = ''

#图像处理参数
Running = True
X_im=0
Y_im=0
c_x=0
c_y=0
c_angle = 0

index = 255
stack_count = 0
# ROS 回调函数，接收话题的消息
def pose_callback(msg):
    global X_im, Y_im ,c_x,c_y
    # 初始化 put 参数为 False
    put=rospy.get_param('put', False)
    if put:
        c_x=msg.pose.position.x
        c_y=msg.pose.position.y
        X_im = msg.pose.position.x
        Y_im = msg.pose.position.y
        process_data()
        # 抓取完成后将 put 参数设置为 False
        rospy.set_param('put', False)
        print("put operation completed. Setting 'put' to False.")

#处理的画面坐标，得到的逆运动学坐标位置
def process_data():
    global theta6
    global X_im,Y_im,kms_x,kms_y,X_mc,Y_mc,c_angle,servo_yuntai,servo_zhuazi,servo_yuntai_bias,Running,systick_ms_bak,next_time,c_x,c_y
    #控制执行下列程序的时间，
    if(int((time.time() * 1000))- systick_ms_bak > next_time):
        systick_ms_bak = int((time.time() * 1000))
        c_bias_x = 50
        c_bias_y = 0
        #将画面坐标换算为逆运动学坐标，由于机械臂在旋转，这里计算的是机械臂坐标系旋转后的坐标位置
        kms_x_c = (c_x-img_w/2+c_bias_x)*30/80
        kms_y_c = 180-((c_y-img_h/2+c_bias_y)*30/80)
        #计算实际坐标，机械臂坐标系旋转之前的坐标位置
        kms_x = kms_x_c*cos(servo_yuntai_bias/(180.0/pi))+kms_y_c*sin(servo_yuntai_bias/(180.0/pi))
        kms_y = -kms_x_c*sin(servo_yuntai_bias/(180.0/pi))+kms_y_c*cos(servo_yuntai_bias/(180.0/pi))
        '''
        输入目标点在图像中的像素坐标，转换为机械臂坐标
        '''

        # 整理两个标定点的坐标
        cali_1_im = [187, 362]                       # 左下角，第一个标定点的像素坐标，要手动填！
        cali_1_mc = [-88, 170]                  # 左下角，第一个标定点的机械臂坐标，要手动填！
        cali_2_im = [492, 119]                        # 右上角，第二个标定点的像素坐标
        cali_2_mc = [110, 260]                    # 右上角，第二个标定点的机械臂坐标，要手动填！270
        
        X_cali_im = [cali_1_im[0], cali_2_im[0]]     # 像素坐标
        X_cali_mc = [cali_1_mc[0], cali_2_mc[0]]     # 机械臂坐标
        Y_cali_im = [cali_2_im[1], cali_1_im[1]]     # 像素坐标，先小后大
        Y_cali_mc = [cali_2_mc[1], cali_1_mc[1]]     # 机械臂坐标，先大后小

        # X差值
        X_mc = int(np.interp(X_im, X_cali_im, X_cali_mc))

        # Y差值
        Y_mc = int(np.interp(Y_im, Y_cali_im, Y_cali_mc))
        #计算云台旋转角度
        if(kms_x == 0 and kms_y != 0) :
            theta6 = 0.0
        elif(kms_y == 0 and kms_x > 0):
            theta6 = 90
        elif(kms_y == 0 and kms_x < 0):
            theta6 = -90
        else :
            theta6 = atan(kms_x/kms_y)*180.0/pi
                  
        servo_yuntai = int(1500-2000.0 * theta6/ 270.0)
                          
        c_angle = abs(c_angle)
        c_angle = c_angle-servo_yuntai_bias                                  
        if theta6>0:                        
            if c_angle <45:
                servo_zhuazi = int(1500-2000.0 * (c_angle+theta6)/ 270.0)
            else:
                angle = 90 -(c_angle+theta6)
                servo_zhuazi = int(1500+2000.0 * (angle)/ 270.0)
            print("++++++++++++++++++++++")
        else:
            if c_angle<45:
                servo_zhuazi = int(1500-2000.0 * (c_angle-abs(theta6))/ 270.0)
            else:
                angle = 90 -(c_angle-abs(theta6))
                servo_zhuazi = int(1500+2000.0 * (angle)/ 270.0)
            print("------------------------")
        debug = False
        if debug:
            print('kms_x:',kms_x,'kms_y:',kms_y)
            print('theta6',theta6)
            print('servo_yuntai_bias',servo_yuntai_bias)
            print('c_angle',c_angle)
            print('servo_yuntai:', servo_yuntai)
            print('servo_zhuazi:', servo_zhuazi)                       
            Str = "#000P{0:0>4d}T{0:0>4d}!#004P{1:0>4d}T{2:0>4d}!#005P1200T{2:0>4d}!".format(servo_yuntai,servo_zhuazi, 1000)
            myUart.uart_send_str(Str)
            time.sleep(3)
            #next_time = 2000
           # myUart.uart_send_str('#000P1500T0500!#004P1500T0500!')
            time.sleep(1)
            kinematics_move(kms_x,kms_y,10,1000)
        else:
            next_time = 8000
            Running = False
            eEvent.set()
        c_x=0
        c_y=0
        X_im = 0
        Y_im = 0
 
                    
#夹取木块
def stcarry_wood():
    global carry_step,Running,run_flag,servo_yuntai,servo_zhuazi,index,servo_yuntai_bias,color_pos_y,stack_count,label,i
    while 1:
        eEvent.wait()       
        print('ok')
        if carry_step == 0:      
            # myUart.uart_send_str("#005P1050T0500!")
            # time.sleep(1)
            carry_step = 1
        elif carry_step == 1:
            Str = "#000P{0:0>4d}T{0:0>4d}!#004P{1:0>4d}T{2:0>4d}!" .format(servo_yuntai,servo_zhuazi, 1000)
            myUart.uart_send_str(Str)  
            time.sleep(1)
            carry_step = 2
                    
        #机械臂运行到目标位置
        elif carry_step == 2:
            # global theta6
            # print('theta6',theta6)
            if X_mc>0:
                kinematics_move(X_mc,Y_mc+14,25,2000)
            else:
                kinematics_move(X_mc-6,Y_mc+14,25,2000)
            time.sleep(3)
            carry_step = 3
        #张开爪子   
        elif carry_step == 3:
            myUart.uart_send_str("#005P1200T1000!")
            time.sleep(2)
            carry_step = 4     
        #抬起机械臂 
        elif carry_step == 4:
            # print("x:{0},y:{1}".format(kms_x,kms_y))
            if X_mc>0:
                kinematics_move(X_mc,Y_mc+14,200,2000)
            else:
                kinematics_move(X_mc-6,Y_mc+14,200,2000)
            time.sleep(1)
            #time.sleep(8)
            myUart.uart_send_str('#000P{0:0>4d}T{1:0>4d}!#001P1500T2000!#002P1800T2000!#003P0700T2000!#004P1500T2000!#005P1500T2000!'.format(int(1500 - 2000 * (servo_yuntai_bias / 270)), 1000))
            time.sleep(2)
            myUart.uart_send_str('#000P{0:0>4d}T{1:0>4d}!#001P1500T2000!#002P1800T2000!#003P0700T2000!#004P1500T2000!#005P1500T2000!'.format(int(1500 - 2000 * (servo_yuntai_bias / 270)), 1000))
            time.sleep(1)
            rospy.set_param('success', True)
            print("Setting 'success' to True.")

            time.sleep(2)
            Running = True        
            carry_step = 0
            run_flag = True
            eEvent.clear()
            
            

#******************************基本函数×××××××××××××××××××××××××
#逆运动学算法
def kinematics_move(x,y,z,mytime): 
    global servo_angle,servo_pwm
    if y < 0 :
        return 0
    #寻找最佳角度
    flag = 0
    my_min = 0
    for i in range(-135,0) :
        if 0 == kms.kinematics_analysis(x,y,z,i):
            if(i<my_min):
                my_min = i
            flag = 1
        
    #用3号舵机与水平最大的夹角作为最佳值
    if(flag) :
        kms.kinematics_analysis(x,y,z,my_min);
        testStr = '{'
        for j in range(0,4) :
            #set_servo(j, servo_pwm[j], time);
            #print(servo_pwm[j])
            testStr += "#%03dP%04dT%04d!" % (j,kms.servo_pwm[j],mytime)
        testStr += '}'
        print(testStr)
        myUart.uart_send_str(testStr)
        return 1
    
    return 0

eEvent = threading.Event()
th1 = threading.Thread(target = stcarry_wood)
th1.setDaemon(True)    
th1.start()

if __name__ == '__main__':
    rospy.init_node('put_node', anonymous=True)
    myUart = WCXC_UartServer.WCXC_UART(115200)    #设置串口 
    kms.setup_kinematics(100, 105, 75, 185)
    rospy.set_param('put', False)
    rospy.set_param('success', False)
    rospy.Subscriber('/Pose', PoseStamped, pose_callback)
    rospy.spin()
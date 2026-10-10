#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy

import actionlib
from actionlib_msgs.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf_conversions import transformations
from math import pi
from std_msgs.msg import Float32MultiArray
import dynamic_reconfigure.client
class navigation_demo:
    def __init__(self):
        self.set_pose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=5)
        #创建一个 actionlib.SimpleActionClient 客户端实例，命名为 self.move_base，
        #用于与名为 "move_base" 的动作服务器进行通信，该服务器处理 MoveBaseAction 类型的动作请求。
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        '''
        调用 wait_for_server 方法，等待动作服务器启动，超时时间为 60 秒。
        如果在 60 秒内服务器没有启动，该方法将返回 False，否则返回 True。'''
        self.move_base.wait_for_server(rospy.Duration(60))

        # 创建一个话题订阅者，订阅名为 'goto_goal' 的话题
        self.goal_sub = rospy.Subscriber('goto_goal', Float32MultiArray, self.goto_callback)

    def set_pose(self, p):
        if self.move_base is None:
            return False

        x, y, th = p
        #PoseWithCovarianceStamped 表示带有协方差的位姿信息，并包含时间戳
        pose = PoseWithCovarianceStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = 'map'
        pose.pose.pose.position.x = x
        pose.pose.pose.position.y = y
        q = transformations.quaternion_from_euler(0.0, 0.0, th/180.0*pi)
        pose.pose.pose.orientation.x = q[0]
        pose.pose.pose.orientation.y = q[1]
        pose.pose.pose.orientation.z = q[2]
        pose.pose.pose.orientation.w = q[3]

        self.set_pose_pub.publish(pose)
        return True

    def _done_cb(self, status, result):
        rospy.loginfo("navigation done! status:%d result:%s"%(status, result))

    def _active_cb(self):
        rospy.loginfo("[Navi] navigation has be actived")

    def _feedback_cb(self, feedback):
        rospy.loginfo("[Navi] navigation feedback\r\n%s"%feedback)

    def goto(self, p):
        goal = MoveBaseGoal()

        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = p[0]
        goal.target_pose.pose.position.y = p[1]
        #将欧拉角转换为四元数，要转换的值为x轴的旋转角度，y轴的旋转角度，z轴的旋转角度
        q = transformations.quaternion_from_euler(0.0, 0.0, p[2]/180.0*pi)
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]

        self.move_base.send_goal(goal, self._done_cb, self._active_cb, self._feedback_cb)
        return True

    def cancel(self):
        self.move_base.cancel_all_goals()
        return True

    def goto_callback(self, msg):
        # 从消息中提取 x, y, yaw 值
        x = msg.data[0]
        y = msg.data[1]
        yaw = msg.data[2]
        rospy.loginfo("Received goal: x=%f, y=%f, yaw=%f", x, y, yaw)
        self.goto([x, y, yaw])

if __name__ == "__main__":
    rospy.init_node('navigation_move',anonymous=True)
    local_obstacle_client = dynamic_reconfigure.client.Client("move_base/local_costmap/obstacle_layer")#初始化客户端
    local_obstacle_config = local_obstacle_client.get_configuration(timeout=8)#保存原始配置
    local_inf_client = dynamic_reconfigure.client.Client("move_base/local_costmap/inflation_layer")
    local_inf_config = local_inf_client.get_configuration(timeout=8)
    global_static_client = dynamic_reconfigure.client.Client("move_base/global_costmap/static_layer")
    global_staitc_config = global_static_client.get_configuration(timeout=8)
    dwa_client = dynamic_reconfigure.client.Client("move_base/DWAPlannerROS")
    dwa_config = dwa_client.get_configuration(timeout=8)


    r = rospy.Rate(0.2)
    rospy.loginfo("set pose...")
    
    navi = navigation_demo()

    # 创建一个话题发布者，发布名为 'goto_goal' 的话题
    goal_pub = rospy.Publisher('goto_goal', Float32MultiArray, queue_size=10)
    rospy.loginfo("setting dynamic parameters ")
    local_obstacle_client.update_configuration({"enabled": 0})
    global_static_client.update_configuration({"enabled": 0})
    local_inf_client.update_configuration({"enabled": 0})
    while not rospy.is_shutdown():
        # 从终端获取输入
        x = float(input("Enter x: "))
        y = float(input("Enter y: "))
        yaw = float(input("Enter yaw: "))

        # 创建消息并发布
        #Float32MultiArray 是 ROS中的一种消息类型，用于存储浮点数数组。
        goal_msg = Float32MultiArray()
        goal_msg.data = [x, y, yaw]
        goal_pub.publish(goal_msg)

        rospy.loginfo("Published goal: x=%f, y=%f, yaw=%f", x, y, yaw)

        # 保持节点运行
        rospy.spin()

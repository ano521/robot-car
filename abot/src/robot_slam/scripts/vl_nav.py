#!/usr/bin/env python3
# -*- coding: utf-8 -*-
'''
Copyright (c) [Zachary]
本代码受版权法保护，未经授权禁止任何形式的复制、分发、修改等使用行为。
Author:Zachary
'''
import rospy
import actionlib
from actionlib_msgs.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf_conversions import transformations
from math import pi
from std_msgs.msg import String
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
import sys
import os
import dynamic_reconfigure.client
import rospy
from vl_locate.srv import GetObject,GetObjectRequest


class navigation_demo:
    def __init__(self):
        #rospy.init_node('navigation_demo', anonymous=True)
        self.set_pose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=5)
        self.arrive_pub = rospy.Publisher('/voiceWords', String, queue_size=10)
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.move_base.wait_for_server(rospy.Duration(60))
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1000)
        rospy.wait_for_service('vlm_detection')
        #self.current_pose_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.current_pose_cb)
        #self.current_pose = None    

    # def current_pose_cb(self, msg):
    #     self.current_pose = msg
    #     if self.current_pose is not None:
    #         self.set_pose_pub.publish(self.current_pose)

    def vlm_detection_client(self,text):
        try:
            # 创建服务代理
            vlm_detection_service = rospy.ServiceProxy('vlm_detection', GetObject)

            # 创建请求对象
            request = GetObjectRequest()
            request.object_name=text
            # 调用服务并获取响应
            rospy.loginfo("调用 'vlm_detection' 服务...")
            response = vlm_detection_service(request)

            # 处理响应
            if response.success:
                rospy.loginfo("服务调用成功！响应消息: %s", response.message)
            else:
                rospy.logwarn("服务调用失败！响应消息: %s", response.message)

        except rospy.ServiceException as e:
            rospy.logerr("服务调用失败: %s", e)

    def rotate(self):
        time1 = 0
        msg = Twist()
        msg.linear.x = 0
        msg.linear.y = 0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 1
        while time1 <= 8:
            self.pub.publish(msg)
            rospy.sleep(0.1)
            time1 += 1
    def end(self):
        time1 = 0
        msg = Twist()
        msg.linear.x = -0.3
        msg.linear.y = 0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0
        while time1 <= 8:
            self.pub.publish(msg)
            rospy.sleep(0.1)
            time1 += 1
    def right(self):
        time1 = 0
        msg = Twist()
        msg.linear.x = 0
        msg.linear.y = -0.5
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0
        while time1 <= 20:
            self.pub.publish(msg)
            rospy.sleep(0.1)
            time1 += 1

    

    def set_pose(self, p):
        if self.move_base is None:
            return False

        x, y, th = p

        pose = PoseWithCovarianceStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = 'map'
        pose.pose.pose.position.x = x
        pose.pose.pose.position.y = y
        q = transformations.quaternion_from_euler(0.0, 0.0, th / 180.0 * pi)
        pose.pose.pose.orientation.x = q[0]
        pose.pose.pose.orientation.y = q[1]
        pose.pose.pose.orientation.z = q[2]
        pose.pose.pose.orientation.w = q[3]

        self.set_pose_pub.publish(pose)
        return True

    def _done_cb(self, status, result):
        rospy.loginfo("navigation done! status:%d result:%s" % (status, result))
        arrive_str = "arrived to target point"
        self.arrive_pub.publish(arrive_str)

    def _active_cb(self):
        rospy.loginfo("[Navi] navigation has been activated")

    def _feedback_cb(self, feedback):
        msg = feedback
        # rospy.loginfo("[Navi] navigation feedback\r\n%s" % feedback)

    def goto(self, p):
        rospy.loginfo("[Navi] goto %s" % p)
        # arrive_str = "going to next point"
        # self.arrive_pub.publish(arrive_str)
        goal = MoveBaseGoal()

        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = p[0]
        goal.target_pose.pose.position.y = p[1]
        q = transformations.quaternion_from_euler(0.0, 0.0, p[2] / 180.0 * pi)
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]

        self.move_base.send_goal(goal, self._done_cb, self._active_cb, self._feedback_cb)
        result = self.move_base.wait_for_result(rospy.Duration(60))
        if not result:
            self.move_base.cancel_goal()
            rospy.loginfo("Timed out achieving goal")
        else:
            state = self.move_base.get_state()
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo("reach goal %s succeeded!" % p)
        return True

    def cancel(self):
        self.move_base.cancel_all_goals()
        return True


if __name__ == "__main__":
    rospy.init_node('navigation_demo', anonymous=True)

    goalListX = rospy.get_param('~goalListX', '2.0, 2.0')
    goalListY = rospy.get_param('~goalListY', '2.0, 4.0')
    goalListYaw = rospy.get_param('~goalListYaw', '0, 90.0')
    goals = [[float(x), float(y), float(yaw)] for (x, y, yaw) in zip(goalListX.split(","), goalListY.split(","), goalListYaw.split(","))]
    input = input("Please enter 1 to continue: ")
    print(goals)
    r = rospy.Rate(1)
    navi = navigation_demo()
    if input == '1':
        local_obstacle_client = dynamic_reconfigure.client.Client("move_base/local_costmap/obstacle_layer")#初始化客户端
        local_obstacle_config = local_obstacle_client.get_configuration(timeout=8)#保存原始配置
        local_inf_client = dynamic_reconfigure.client.Client("move_base/local_costmap/inflation_layer")
        local_inf_config = local_inf_client.get_configuration(timeout=8)
        global_static_client = dynamic_reconfigure.client.Client("move_base/global_costmap/static_layer")
        global_staitc_config = global_static_client.get_configuration(timeout=8)
        dwa_client = dynamic_reconfigure.client.Client("move_base/DWAPlannerROS")
        dwa_config = dwa_client.get_configuration(timeout=8)
        # rospy.loginfo("setting dynamic parameters to avoid collision")
        # dwa_client.update_configuration({"max_vel_x": 4, "min_vel_x": -4, "max_vel_y": 4, "min_vel_y": -4, "max_vel_trans": 4, "min_vel_trans": -4})
        # local_obstacle_client.update_configuration({"enabled": 0})
        # global_static_client.update_configuration({"enabled": 0})
        # local_inf_client.update_configuration({"enabled": 0})

        # rospy.sleep(5)
        rospy.loginfo("setting dynamic parameters ")
        local_obstacle_client.update_configuration({"enabled": 0})
        global_static_client.update_configuration({"enabled": 0})
        local_inf_client.update_configuration({"enabled": 0})
        navi.goto(goals[1])
        rospy.sleep(5) 
        rospy.set_param('/grab', 1)
        navi.vlm_detection_client("自行车")
        while True:
            success=rospy.get_param('/success', False)
            if success:
                rospy.loginfo("抓取成功，进行下一步操作")
                success = False  # 重置标志位
                rospy.set_param('/success', False)
                break
        rospy.sleep(5)
        navi.end()
        navi.goto(goals[2])
        rospy.sleep(3) 
        rospy.set_param('/put', 1)
        navi.vlm_detection_client("自行车")
        while True:
            success=rospy.get_param('/success', False)
            if success:
                rospy.loginfo("放置成功，进行下一步操作")
                success = False  # 重置标志位
                rospy.set_param('/success', False)
                break
        navi.goto(goals[3])


        navi.goto(goals[4])
        rospy.sleep(5) 
        rospy.set_param('/grab', 1)
        navi.vlm_detection_client("钟表")
        while True:
            success=rospy.get_param('/success', False)
            if success:
                rospy.loginfo("抓取成功，进行下一步操作")
                success = False  # 重置标志位
                rospy.set_param('/success', False)
                break
        rospy.sleep(5)
        navi.end()
        navi.goto(goals[10])
        navi.goto(goals[5])
        rospy.sleep(3) 
        rospy.set_param('/put', 1)
        navi.vlm_detection_client("钟表")
        while True:
            success=rospy.get_param('/success', False)
            if success:
                rospy.loginfo("放置成功，进行下一步操作")
                success = False  # 重置标志位
                rospy.set_param('/success', False)
                break
        navi.goto(goals[6])

        navi.goto(goals[7])
        rospy.sleep(5) 
        rospy.set_param('/grab', 1)
        navi.vlm_detection_client("汽车")
        while True:
            success=rospy.get_param('/success', False)
            if success:
                rospy.loginfo("抓取成功，进行下一步操作")
                success = False  # 重置标志位
                rospy.set_param('/success', False)
                break
        rospy.sleep(5)
        navi.end()
        navi.goto(goals[8])
        rospy.sleep(3) 
        rospy.set_param('/put', 1)
        navi.vlm_detection_client("汽车")
        while True:
            success=rospy.get_param('/success', False)
            if success:
                rospy.loginfo("放置成功，进行下一步操作")
                success = False  # 重置标志位
                rospy.set_param('/success', False)
                break
        navi.end()
        navi.goto(goals[11])
        navi.goto(goals[12])



        # rospy.loginfo("setting dynamic parameters ")
        # local_obstacle_client.update_configuration({"enabled": 0})
        # global_static_client.update_configuration({"enabled": 0})
        # local_inf_client.update_configuration({"enabled": 0})
        # navi.goto(goals[3])#进点
        # rospy.sleep(2)
        # rospy.set_param('grab', True)
        # rospy.sleep(10)
        # rospy.loginfo("grap completed")
        # navi.goto(goals[int(end_result)+11])#到达邻近点
        # rospy.sleep(2)
        # #恢复参数
        # local_obstacle_client.update_configuration({"enabled": local_obstacle_config["enabled"]})
        # global_static_client.update_configuration({"enabled": global_staitc_config["enabled"]})
        # local_inf_client.update_configuration({"enabled": local_inf_config["enabled"]})
        # navi.goto(goals[14])#到达终点
        # rospy.set_param('put', True)

        # while not rospy.is_shutdown():
        #     r.sleep()

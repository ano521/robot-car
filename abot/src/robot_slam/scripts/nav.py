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
import rospy
from std_srvs.srv import Trigger
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
import sys
import os
import dynamic_reconfigure.client
from std_srvs.srv import Empty


class navigation_demo:
    def __init__(self):
        #rospy.init_node('navigation_demo', anonymous=True)
        self.set_pose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=5)
        self.arrive_pub = rospy.Publisher('/voiceWords', String, queue_size=10)
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.move_base.wait_for_server(rospy.Duration(60))
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1000)
        #self.current_pose_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.current_pose_cb)
        #self.current_pose = None    

    # def current_pose_cb(self, msg):
    #     self.current_pose = msg
    #     if self.current_pose is not None:
    #         self.set_pose_pub.publish(self.current_pose)
    def toggle_tracking(self):
        """
        目标跟踪状态
        """
        try:
            rospy.wait_for_service('start_target_tracking', timeout=5.0)
            service = rospy.ServiceProxy('start_target_tracking', Trigger)
            response = service()
            print(response.message)
            return response.success
        except Exception as e:
            print(f"错误: {e}")
            return False
        
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
    def call_global_localization(self):
        rospy.wait_for_service('/global_localization')
        try:
            global_localization = rospy.ServiceProxy('/global_localization', Empty)
            global_localization()
            rospy.loginfo("Global localization initiated.")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")


if __name__ == "__main__":
    rospy.init_node('navigation_demo', anonymous=True)

    goalListX = rospy.get_param('~goalListX', '2.0, 2.0')
    goalListY = rospy.get_param('~goalListY', '2.0, 4.0')
    goalListYaw = rospy.get_param('~goalListYaw', '0, 90.0')
    goals = [[float(x), float(y), float(yaw)] for (x, y, yaw) in zip(goalListX.split(","), goalListY.split(","), goalListYaw.split(","))]
    print('Please 1 to continue: ')
    input = input("Please enter 1 to continue: ")
    print(goals)
    r = rospy.Rate(1)
    navi = navigation_demo()
    #navi.call_global_localization()
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
        # ------------------------------------------------------------------
        # 【2026-10-12 修改 · 安全】下面三行原本是打开的（与 pub_point.py 是同一处隐患，
        #   即问题汇总里的 D9）。它们会把 move_base 的
        #   局部代价地图 · 障碍物层  (local_costmap/obstacle_layer)
        #   全局代价地图 · 静态地图层(global_costmap/static_layer)
        #   局部代价地图 · 膨胀层    (local_costmap/inflation_layer)
        # 全部设成 enabled=0 —— 代价地图里不再有任何障碍物，规划出来的路径会
        # 直接穿墙、紧贴墙走。
        # ★ 本文件由 grasp.sh / 4-test.sh 通过 `roslaunch robot_slam grab.launch` 启动，
        #   也就是说【一按 1 就会把避障关掉】，然后开始往写死的 16 个点位跑，
        #   中途还有 3 处 navi.end() 不看传感器的盲退（-0.3 m/s × 8 s ≈ 2.4 m）。
        #   D9 当时只修了 pub_point.py（3-amcl_test.sh 走的那条），漏了这里。
        # 如需恢复厂商原行为，把下面三行的 # 去掉即可。
        # ------------------------------------------------------------------
        # local_obstacle_client.update_configuration({"enabled": 0})
        # global_static_client.update_configuration({"enabled": 0})
        # local_inf_client.update_configuration({"enabled": 0})
        navi.goto(goals[12])
        navi.goto(goals[0])
        # navi.toggle_tracking()
        # while True:
        #     success=rospy.get_param('/tiaozheng', False)
        #     if success:
        #         rospy.loginfo("调整成功")
        #         success = False  # 重置标志位
        #         rospy.set_param('/tiaozheng', False)
        #         break
        rospy.sleep(2) 
        rospy.set_param('/grab', 1)
        while True:
            success=rospy.get_param('/success', False)
            if success:
                rospy.loginfo("抓取成功，进行下一步操作")
                success = False  # 重置标志位
                rospy.set_param('/success', False)
                break
        #rospy.sleep(3)
        navi.end()
        navi.goto(goals[14])
        navi.goto(goals[1])
        # navi.toggle_tracking()
        rospy.sleep(2) 
        rospy.set_param('/put', 1)
        while True:
            success=rospy.get_param('/success', False)
            if success:
                rospy.loginfo("放置成功，进行下一步操作")
                success = False  # 重置标志位
                rospy.set_param('/success', False)
                break
        navi.end()
        navi.goto(goals[2])
        navi.goto(goals[3])
        navi.goto(goals[13])
        navi.goto(goals[4])
        # navi.toggle_tracking()
        rospy.sleep(2) 
        rospy.set_param('/grab', 1)
        while True:
            success=rospy.get_param('/success', False)
            if success:
                rospy.loginfo("抓取成功，进行下一步操作")
                success = False  # 重置标志位
                rospy.set_param('/success', False)
                break
        navi.end()
        navi.goto(goals[5])
        # navi.toggle_tracking()
        rospy.sleep(2) 
        rospy.set_param('/put', 1)
        while True:
            success=rospy.get_param('/success', False)
            if success:
                rospy.loginfo("放置成功，进行下一步操作")
                success = False  # 重置标志位
                rospy.set_param('/success', False)
                break
        navi.end()
        navi.goto(goals[6])
        navi.goto(goals[7])
        navi.goto(goals[8])
        # navi.toggle_tracking()
        rospy.sleep(2) 
        rospy.set_param('/grab', 1)
        while True:
            success=rospy.get_param('/success', False)
            if success:
                rospy.loginfo("抓取成功，进行下一步操作")
                success = False  # 重置标志位
                rospy.set_param('/success', False)
                break
        #rospy.sleep(5)
        navi.end()
        navi.goto(goals[9])
        # navi.toggle_tracking()
        rospy.sleep(2) 
        rospy.set_param('/put', 1)
        while True:
            success=rospy.get_param('/success', False)
            if success:
                rospy.loginfo("放置成功，进行下一步操作")
                success = False  # 重置标志位
                rospy.set_param('/success', False)
                break
        navi.goto(goals[10])
        navi.goto(goals[11])
        #rospy.sleep(5) 
        # rospy.set_param('/grab', 1)
        # while True:
        #     success=rospy.get_param('/success', False)
        #     if success:
        #         rospy.loginfo("抓取成功，进行下一步操作")
        #         success = False  # 重置标志位
        #         rospy.set_param('/success', False)
        #         break
        # rospy.sleep(5)
        # navi.end()
        # navi.goto(goals[8])
        # rospy.sleep(3) 
        # rospy.set_param('/put', 1)
        # while True:
        #     success=rospy.get_param('/success', False)
        #     if success:
        #         rospy.loginfo("放置成功，进行下一步操作")
        #         success = False  # 重置标志位
        #         rospy.set_param('/success', False)
        #         break
        # navi.end()
        # navi.goto(goals[11])
        # navi.goto(goals[12])



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

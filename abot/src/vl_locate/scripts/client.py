#!/usr/bin/env python3
# -*- coding: utf-8 -*-
'''
Copyright (c) [Zachary]
本代码受版权法保护，未经授权禁止任何形式的复制、分发、修改等使用行为。
Author:Zachary
company:WCXC
'''
import rospy
from vl_locate.srv import GetObject,GetObjectRequest

def vlm_detection_client(text):
    # 初始化ROS节点
    rospy.init_node('vlm_detection_client')

    # 等待服务可用
    rospy.loginfo("等待服务 'vlm_detection' 可用...")
    rospy.wait_for_service('vlm_detection')

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

if __name__ == '__main__':
    try:
        rospy.set_param('grab', True)
        vlm_detection_client("自行车")
        while True:
            success=rospy.get_param('/success', False)
            if success:
                rospy.loginfo("抓取成功，进行下一步操作")
                success = False  # 重置标志位
                rospy.set_param('/success', False)
                break
        rospy.set_param('put', True)
        vlm_detection_client("汽车")
        while True:
            success=rospy.get_param('/success', False)
            if success:
                rospy.loginfo("放置成功，进行下一步操作")
                success = False  # 重置标志位
                rospy.set_param('/success', False)
                break
    except rospy.ROSInterruptException:
        pass
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# 2026-10-28 修 D3：原文件 find_cb 被缩进在 __init__ 里面，
# rospy.Subscriber(..., self.find_cb) 拿不到属性 → 一构造就 AttributeError；
# 末尾 try/except 的缩进也是坏的（except 比 try 还深），整份文件连语法都不成立。
# 现在 find_cb 已反缩进成类方法，其余行为保持原样：
# 订阅 /object_position (geometry_msgs/Point)，把 x/y/z 打到终端。
import rospy
from geometry_msgs.msg import Point


class object_position(object):
    def __init__(self):
        rospy.init_node('object_position_node', anonymous=True)
        self.find_sub = rospy.Subscriber('/object_position', Point, self.find_cb)

    def find_cb(self, data):
        print('x:', data.x)
        print('y:', data.y)
        print('z:', data.z)


if __name__ == '__main__':
    try:
        object_position()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

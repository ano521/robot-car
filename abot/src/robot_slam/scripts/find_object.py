#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Point

class object_position:
    def __init__(self):
        rospy.init_node('object_position_node', anonymous=True)
        self.find_sub = rospy.Subscriber('/object_position', Point, self.find_cb);
        def find_cb(self, data):
            point_msg = data
            print('x:', point_msg.x)
            print('y:', point_msg.y)
            print('z:', point_msg.z)
if __name__ == '__main__':
            try:
                  object_position = object_position()
                  rospy.spin()
                  except rospy.ROSInterruptException:
pass

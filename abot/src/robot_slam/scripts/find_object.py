#!/usr/bin/env python3
"""Report template-match positions published by print_objects_detected."""

import rospy
from geometry_msgs.msg import Point


class ObjectPositionReporter:
    def __init__(self):
        self.position_topic = rospy.get_param(
            "~position_topic", "/object_position"
        )
        self.output_topic = rospy.get_param(
            "~output_topic", "/vision/template_position"
        )
        self.target_object_id = int(
            rospy.get_param("~target_object_id", -1)
        )
        self.publisher = rospy.Publisher(
            self.output_topic, Point, queue_size=10
        )
        rospy.Subscriber(
            self.position_topic, Point, self.position_callback
        )

    def position_callback(self, point_message):
        object_id = int(round(point_message.z))
        if object_id == 255:
            rospy.loginfo_throttle(1.0, "No template object detected")
            return
        if (
            self.target_object_id >= 0
            and object_id != self.target_object_id
        ):
            return

        self.publisher.publish(point_message)
        rospy.loginfo_throttle(
            0.5,
            "Detected template object_id=%d position=(%.1f, %.1f)",
            object_id,
            point_message.x,
            point_message.y,
        )


if __name__ == "__main__":
    rospy.init_node("template_object_reporter", anonymous=False)
    try:
        ObjectPositionReporter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

#!/usr/bin/env python3
"""Report AR-tag detection details from ar_track_alvar."""

import rospy
from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import Point


class ARTrackerReporter:
    def __init__(self):
        self.markers_topic = rospy.get_param(
            "~markers_topic", "/ar_pose_marker"
        )
        self.output_topic = rospy.get_param(
            "~output_topic", "/vision/ar_position"
        )
        self.target_marker_id = int(
            rospy.get_param("~target_marker_id", -1)
        )
        self.publisher = rospy.Publisher(
            self.output_topic, Point, queue_size=10
        )
        rospy.Subscriber(
            self.markers_topic, AlvarMarkers, self.markers_callback
        )

    def markers_callback(self, message):
        markers = [
            marker
            for marker in message.markers
            if self.target_marker_id < 0
            or marker.id == self.target_marker_id
        ]
        if not markers:
            rospy.loginfo_throttle(1.0, "No matching AR marker detected")
            return

        for marker in markers:
            position = marker.pose.pose.position
            self.publisher.publish(
                Point(x=position.x, y=position.y, z=marker.id)
            )
            rospy.loginfo_throttle(
                0.5,
                "Detected AR marker_id=%d position=(%.3f, %.3f, %.3f)",
                marker.id,
                position.x,
                position.y,
                position.z,
            )


if __name__ == "__main__":
    rospy.init_node("ar_tracker_reporter", anonymous=False)
    try:
        ARTrackerReporter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

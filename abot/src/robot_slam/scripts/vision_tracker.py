#!/usr/bin/env python3
"""Track a template or AR marker by rotating the base toward its center."""

import math
import sys

import rospy
from ar_track_alvar_msgs.msg import AlvarMarkers
from find_object_2d.msg import ObjectsStamped
from geometry_msgs.msg import Point, Twist
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Float32, String

from vision_common import (
    angular_velocity_for_error,
    normalized_bearing,
    normalized_pixel_offset,
    parse_template_detections,
    select_template_detection,
    smooth,
)


class VisionTracker:
    def __init__(self, default_source="template"):
        self.source = rospy.get_param("~source", default_source).lower()
        if self.source not in ("template", "ar"):
            raise ValueError("~source must be 'template' or 'ar'")

        self.objects_topic = rospy.get_param(
            "~objects_topic", "/objectsStamped"
        )
        self.ar_topic = rospy.get_param("~ar_topic", "/ar_pose_marker")
        self.camera_info_topic = rospy.get_param(
            "~camera_info_topic", "/usb_cam/camera_info"
        )
        self.cmd_vel_topic = rospy.get_param("~cmd_vel_topic", "/cmd_vel")
        self.target_topic = rospy.get_param(
            "~target_topic", "/vision_tracking/target"
        )
        self.error_topic = rospy.get_param(
            "~error_topic", "/vision_tracking/error"
        )
        self.status_topic = rospy.get_param(
            "~status_topic", "/vision_tracking/status"
        )

        self.target_object_id = int(rospy.get_param("~target_object_id", -1))
        self.target_marker_id = int(rospy.get_param("~target_marker_id", -1))
        self.image_width = int(rospy.get_param("~image_width", 640))
        self.horizontal_fov_degrees = float(
            rospy.get_param("~horizontal_fov_degrees", 62.0)
        )
        self.use_camera_info_fov = bool(
            rospy.get_param("~use_camera_info_fov", True)
        )
        self.deadband = float(rospy.get_param("~deadband", 0.06))
        self.angular_gain = float(rospy.get_param("~angular_gain", 0.8))
        self.max_angular_speed = float(
            rospy.get_param("~max_angular_speed", 0.35)
        )
        self.smoothing_alpha = float(
            rospy.get_param("~smoothing_alpha", 0.35)
        )
        self.lost_timeout = float(rospy.get_param("~lost_timeout", 0.6))
        self.rate_hz = float(rospy.get_param("~rate_hz", 20.0))
        self.enable_motion_default = bool(
            rospy.get_param("~enable_motion", False)
        )

        self.last_error = None
        self.last_target_time = rospy.Time.now()
        self.target_visible = False
        self.last_status = None
        self.commanded_motion = False

        self.cmd_pub = rospy.Publisher(
            self.cmd_vel_topic, Twist, queue_size=1
        )
        self.target_pub = rospy.Publisher(
            self.target_topic, Point, queue_size=1
        )
        self.error_pub = rospy.Publisher(
            self.error_topic, Float32, queue_size=1
        )
        self.status_pub = rospy.Publisher(
            self.status_topic, String, queue_size=1
        )

        rospy.Subscriber(
            self.camera_info_topic, CameraInfo, self.camera_info_callback
        )
        if self.source == "template":
            rospy.Subscriber(
                self.objects_topic, ObjectsStamped, self.objects_callback
            )
        else:
            rospy.Subscriber(
                self.ar_topic, AlvarMarkers, self.ar_callback
            )

        rospy.on_shutdown(lambda: self.stop(force=True))
        rospy.Timer(rospy.Duration(1.0 / self.rate_hz), self.control_loop)
        rospy.loginfo(
            "Vision tracker started: source=%s, enable_motion=%s",
            self.source,
            self.enable_motion_default,
        )

    def camera_info_callback(self, message):
        if message.width > 0:
            self.image_width = int(message.width)
        if (
            self.use_camera_info_fov
            and message.width > 0
            and len(message.K) >= 1
            and message.K[0] > 0.0
        ):
            self.horizontal_fov_degrees = math.degrees(
                2.0 * math.atan(message.width / (2.0 * message.K[0]))
            )

    def objects_callback(self, message):
        detections = parse_template_detections(message.objects.data)
        detection = select_template_detection(
            detections, self.target_object_id
        )
        if detection is None:
            self.mark_lost("template not found")
            return

        error = normalized_pixel_offset(
            detection.center_x, self.image_width
        )
        self.mark_target(error)
        self.target_pub.publish(
            Point(
                x=detection.center_x,
                y=detection.center_y,
                z=detection.object_id,
            )
        )
        rospy.loginfo_throttle(
            1.0,
            "Template id=%d center=(%.1f, %.1f), error=%+.3f",
            detection.object_id,
            detection.center_x,
            detection.center_y,
            error,
        )

    def ar_callback(self, message):
        markers = [
            marker
            for marker in message.markers
            if self.target_marker_id < 0
            or marker.id == self.target_marker_id
        ]
        if not markers:
            self.mark_lost("AR marker not found")
            return

        marker = min(
            markers,
            key=lambda candidate: math.hypot(
                candidate.pose.pose.position.x,
                candidate.pose.pose.position.z,
            ),
        )
        position = marker.pose.pose.position
        bearing = math.atan2(position.x, position.z)
        error = normalized_bearing(
            bearing, self.horizontal_fov_degrees
        )
        self.mark_target(error, reset_filter=True)
        self.target_pub.publish(
            Point(x=bearing, y=position.y, z=marker.id)
        )
        rospy.loginfo_throttle(
            1.0,
            "AR id=%d xyz=(%.3f, %.3f, %.3f), bearing=%+.3f",
            marker.id,
            position.x,
            position.y,
            position.z,
            bearing,
        )

    def mark_target(self, error, reset_filter=False):
        if reset_filter:
            self.last_error = None
        self.last_error = smooth(
            self.last_error, error, self.smoothing_alpha
        )
        self.last_target_time = rospy.Time.now()
        self.target_visible = True

    def mark_lost(self, reason):
        if self.target_visible:
            rospy.logwarn_throttle(1.0, "Tracking target lost: %s", reason)
        self.target_visible = False
        self.last_error = None

    def control_loop(self, _event):
        motion_enabled = bool(
            rospy.get_param("~enable_motion", self.enable_motion_default)
        )
        age = (rospy.Time.now() - self.last_target_time).to_sec()
        has_fresh_target = (
            self.target_visible
            and self.last_error is not None
            and age <= self.lost_timeout
        )

        if not motion_enabled:
            self.stop()
            self.set_status("motion disabled")
            return
        if not has_fresh_target:
            self.stop(force=True)
            self.error_pub.publish(Float32(float("nan")))
            self.set_status("target lost")
            return

        angular = angular_velocity_for_error(
            self.last_error,
            self.deadband,
            self.angular_gain,
            self.max_angular_speed,
        )
        command = Twist()
        command.angular.z = angular
        self.cmd_pub.publish(command)
        self.commanded_motion = angular != 0.0
        self.error_pub.publish(Float32(self.last_error))
        self.set_status("centered" if angular == 0.0 else "tracking")

    def stop(self, force=False):
        if hasattr(self, "cmd_pub") and (force or self.commanded_motion):
            self.cmd_pub.publish(Twist())
            self.commanded_motion = False

    def set_status(self, status):
        if status != self.last_status:
            self.last_status = status
            self.status_pub.publish(String(data=status))


def main(default_source="template"):
    rospy.init_node("vision_tracker", anonymous=False)
    try:
        VisionTracker(default_source)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    selected_source = sys.argv[1] if len(sys.argv) > 1 else "template"
    main(selected_source)

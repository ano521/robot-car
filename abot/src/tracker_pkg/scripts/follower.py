#!/usr/bin/env python3

import time

import numpy as np
import rospy
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64


class Follower:
    def __init__(self):
        self.max_speed = float(rospy.get_param("~maxSpeed", 0.4))
        self.active = False
        self.cmd_vel_publisher = rospy.Publisher(
            "/cmd_vel", Twist, queue_size=3
        )

        self.joy_subscriber = rospy.Subscriber(
            "/joy", Joy, self.joy_callback
        )
        self.robot_enable_button = int(
            rospy.get_param("~enable_button", 4)
        )
        self.buttons = []

        self.position_subscriber = rospy.Subscriber(
            "/object_tracker/angle_X", Float64, self.position_callback
        )
        pid_parameters = rospy.get_param("~PID_controller")
        self.pid_controller = SimplePID(
            0.0,
            pid_parameters["P"],
            pid_parameters["I"],
            pid_parameters["D"],
        )
        rospy.on_shutdown(self.shutdown)

    def position_callback(self, angle_message):
        angle_x = float(angle_message.data)
        unclipped_angular_speed = self.pid_controller.update(angle_x)
        angular_speed = float(
            np.clip(
                -unclipped_angular_speed,
                -self.max_speed,
                self.max_speed,
            )
        )
        velocity = Twist()
        velocity.linear = Vector3(0.0, 0.0, 0.0)
        velocity.angular = Vector3(0.0, 0.0, angular_speed)
        rospy.loginfo("angular speed: %.3f", angular_speed)

        joystick_enabled = (
            len(self.buttons) > self.robot_enable_button
            and self.buttons[self.robot_enable_button] == 1
        )
        if joystick_enabled:
            self.cmd_vel_publisher.publish(velocity)
        else:
            rospy.loginfo_throttle(
                2.0,
                "Joystick enable button %d is not pressed; motion blocked",
                self.robot_enable_button,
            )
            self.stop_moving()

    def stop_moving(self):
        self.cmd_vel_publisher.publish(Twist())

    def shutdown(self):
        self.stop_moving()
        self.active = False
        rospy.loginfo("stop moving")

    def joy_callback(self, message):
        self.buttons = list(message.buttons)


class SimplePID:
    def __init__(self, target, proportional, integral, derivative):
        shapes_match = (
            np.size(proportional)
            == np.size(integral)
            == np.size(derivative)
        )
        if not shapes_match:
            raise TypeError("PID parameter shapes are not compatible")

        rospy.loginfo(
            "PID initialised with P:%s, I:%s, D:%s",
            proportional,
            integral,
            derivative,
        )
        self.kp = np.array(proportional)
        self.ki = np.array(integral)
        self.kd = np.array(derivative)
        self.set_point = np.array(target)
        self.last_error = 0
        self.integrator = 0
        self.time_of_last_call = None

    def update(self, current_value):
        current_value = np.array(current_value)
        if np.size(current_value) != np.size(self.set_point):
            raise TypeError(
                "current_value and target do not have the same shape"
            )
        if self.time_of_last_call is None:
            self.time_of_last_call = time.monotonic()
            return np.zeros(np.size(current_value))

        error = self.set_point - current_value
        current_time = time.monotonic()
        delta_time = max(current_time - self.time_of_last_call, 1e-6)
        self.integrator += error * delta_time
        derivative = (error - self.last_error) / delta_time
        self.last_error = error
        self.time_of_last_call = current_time
        return (
            self.kp * error
            + self.ki * self.integrator
            + self.kd * derivative
        )


if __name__ == "__main__":
    rospy.init_node("follower")
    follower = Follower()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

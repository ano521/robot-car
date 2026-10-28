#!/usr/bin/env python3
"""Offline tests for ai_locate.py with lightweight ROS module fakes."""

import importlib.util
import sys
import types
import unittest
from pathlib import Path


SCRIPTS = Path(__file__).resolve().parents[1] / "script"
AI_LOCATE = SCRIPTS / "ai_locate.py"


class FakePublisher:
    def __init__(self, topic, message_type, queue_size=0):
        self.topic = topic
        self.message_type = message_type
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class FakeSubscriber:
    def __init__(self, topic, message_type, callback, queue_size=0):
        self.topic = topic
        self.message_type = message_type
        self.callback = callback


class Float32MultiArray:
    def __init__(self, data=None):
        self.data = [] if data is None else data


class Int32MultiArray:
    def __init__(self, data=None):
        self.data = [] if data is None else data


class Detection:
    def __init__(self):
        self.class_name = ""
        self.confidence = 0.0
        self.x1 = 0.0
        self.y1 = 0.0
        self.x2 = 0.0
        self.y2 = 0.0


def install_fake_ros():
    rospy = types.ModuleType("rospy")
    rospy.params = {}
    rospy.get_param = lambda name, default=None: rospy.params.get(name, default)
    rospy.Publisher = FakePublisher
    rospy.Subscriber = FakeSubscriber
    rospy.loginfo = lambda *args, **kwargs: None
    rospy.logdebug_throttle = lambda *args, **kwargs: None

    std_msgs = types.ModuleType("std_msgs")
    std_msgs_msg = types.ModuleType("std_msgs.msg")
    std_msgs_msg.Float32MultiArray = Float32MultiArray
    std_msgs_msg.Int32MultiArray = Int32MultiArray

    yolov8_ros = types.ModuleType("yolov8_ros")
    yolov8_ros_msg = types.ModuleType("yolov8_ros.msg")
    yolov8_ros_msg.Detection = Detection

    sys.modules["rospy"] = rospy
    sys.modules["std_msgs"] = std_msgs
    sys.modules["std_msgs.msg"] = std_msgs_msg
    sys.modules["yolov8_ros"] = yolov8_ros
    sys.modules["yolov8_ros.msg"] = yolov8_ros_msg
    return rospy


def load_ai_locate():
    spec = importlib.util.spec_from_file_location("ai_locate_offline", AI_LOCATE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class AiLocateOfflineTest(unittest.TestCase):
    def setUp(self):
        self.rospy = install_fake_ros()
        self.module = load_ai_locate()

    def make_node(self, **params):
        self.rospy.params = {
            "~only_id": -1,
            "~min_area_px": 0,
            "~confidence": 1.0,
            "~max_abs_px": 10000.0,
            "~objects_topic": "/objects",
        }
        self.rospy.params.update(params)
        return self.module.AiLocate()

    def test_identity_homography_publishes_detection(self):
        node = self.make_node()
        node.cb(
            Float32MultiArray(
                [4, 100, 80, 1, 0, 0, 0, 1, 0, 0, 0, 1]
            )
        )

        self.assertEqual(len(node.pub.messages), 1)
        detection = node.pub.messages[0]
        self.assertEqual(detection.class_name, "4")
        self.assertEqual((detection.x1, detection.y1), (0.0, 0.0))
        self.assertEqual((detection.x2, detection.y2), (100.0, 80.0))
        self.assertEqual(node.pub_ids.messages[-1].data, [4])

    def test_only_id_filters_other_markers(self):
        node = self.make_node(**{"~only_id": 9})
        node.cb(
            Float32MultiArray(
                [4, 10, 10, 1, 0, 0, 0, 1, 0, 0, 0, 1]
            )
        )

        self.assertEqual(node.pub.messages, [])
        self.assertEqual(node.pub_ids.messages[-1].data, [])

    def test_empty_payload_publishes_empty_ids(self):
        node = self.make_node()
        node.cb(Float32MultiArray([]))
        self.assertEqual(node.pub.messages, [])
        self.assertEqual(node.pub_ids.messages[-1].data, [])

    def test_malformed_trailing_values_are_ignored(self):
        node = self.make_node()
        node.cb(Float32MultiArray([4, 10, 10, 1]))
        self.assertEqual(node.pub.messages, [])
        self.assertEqual(node.pub_ids.messages[-1].data, [])

    def test_small_area_is_filtered(self):
        node = self.make_node(**{"~min_area_px": 1000})
        node.cb(
            Float32MultiArray(
                [2, 10, 10, 1, 0, 0, 0, 1, 0, 0, 0, 1]
            )
        )
        self.assertEqual(node.pub.messages, [])
        self.assertEqual(node.pub_ids.messages[-1].data, [])


if __name__ == "__main__":
    unittest.main()

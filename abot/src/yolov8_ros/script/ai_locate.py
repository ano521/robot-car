#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ai_locate.py —— 把 find_object_2d 的模板匹配结果，翻译成现有抓取链能吃的 Detection 消息

为什么要它：
    find_object_2d 是 C++ 的 ORB 特征匹配（抗旋转、抗缩放，正好对付"物块随手摆"），
    它发的是 /objects（std_msgs/Float32MultiArray），格式很原始；
    而我们的 grasp_once.py 只认 /yolov8_detections（yolov8_ros/Detection）。
    这个节点只干一件事：翻译。**grasp_once.py 一行都不用改。**

/objects 格式（每个目标 12 个 float，来自 abot_find/src/ros/print_objects_detected_node.cpp）：
    [i+0]  id（就是模板文件名去掉扩展名后的正整序号：4.png -> 4）
    [i+1]  模板图宽 W
    [i+2]  模板图高 H
    [i+3 .. i+11]  单应矩阵，Qt 的 QTransform 顺序：
                   m11 m12 m13  m21 m22 m23  m31 m32 m33
    把模板的 (0,0) (W,0) (0,H) (W,H) 四个角投到图像上，就得到 bbox。

用法：
    rosrun yolov8_ros ai_locate.py
    rosrun yolov8_ros ai_locate.py _only_id:=2      # 只转发 2 号
    rostopic echo /ai_locate/ids -n1                # 看当前画面里有哪几号（识别阶段用）

配合：
    roslaunch find_object_2d find_object_2d.launch gui:=true \
        objects_path:=/media/abot/999/abot/src/abot_find/object_task \
        settings_path:=/media/abot/999/abot/_ros/find_object_2d.ini
    （settings_path 必须指到工作区，别让它写 /home/abot）
"""

import rospy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int32MultiArray

from yolov8_ros.msg import Detection

FLOATS_PER_OBJECT = 12


class AiLocate(object):
    def __init__(self):
        self.only_id = int(rospy.get_param('~only_id', -1))
        self.min_area = float(rospy.get_param('~min_area_px', 400))
        self.score = float(rospy.get_param('~confidence', 1.0))
        self.max_abs = float(rospy.get_param('~max_abs_px', 10000.0))

        self.pub = rospy.Publisher('/yolov8_detections', Detection, queue_size=10)
        self.pub_ids = rospy.Publisher('/ai_locate/ids', Int32MultiArray, queue_size=10)
        self.sub = rospy.Subscriber(rospy.get_param('~objects_topic', '/objects'),
                                    Float32MultiArray, self.cb, queue_size=10)

        rospy.loginfo("ai_locate 就绪：only_id=%s  min_area=%s  ->  /yolov8_detections"
                      % ("全部" if self.only_id < 0 else self.only_id, self.min_area))

    @staticmethod
    def _map(m, x, y):
        """Qt QTransform 的 map 语义（投影变换）"""
        den = m[2] * x + m[5] * y + m[8]
        if abs(den) < 1e-9:
            return None
        return ((m[0] * x + m[3] * y + m[6]) / den,
                (m[1] * x + m[4] * y + m[7]) / den)

    def cb(self, msg):
        d = msg.data
        if not d or len(d) < FLOATS_PER_OBJECT:
            self.pub_ids.publish(Int32MultiArray(data=[]))
            return

        ids = []
        n_ok = 0
        for i in range(0, len(d) - FLOATS_PER_OBJECT + 1, FLOATS_PER_OBJECT):
            oid = int(d[i])
            w = float(d[i + 1])
            h = float(d[i + 2])
            m = d[i + 3:i + 12]
            if w <= 0 or h <= 0:
                continue
            if self.only_id >= 0 and oid != self.only_id:
                continue

            corners = [self._map(m, 0, 0), self._map(m, w, 0),
                       self._map(m, 0, h), self._map(m, w, h)]
            if any(c is None for c in corners):
                continue
            xs = [c[0] for c in corners]
            ys = [c[1] for c in corners]
            if max(abs(v) for v in xs + ys) > self.max_abs:
                continue

            x1, x2 = min(xs), max(xs)
            y1, y2 = min(ys), max(ys)
            area = (x2 - x1) * (y2 - y1)
            if area < self.min_area:
                continue

            det = Detection()
            det.class_name = str(oid)
            det.confidence = self.score
            det.x1 = x1
            det.y1 = y1
            det.x2 = x2
            det.y2 = y2
            self.pub.publish(det)
            ids.append(oid)
            n_ok += 1
            rospy.loginfo("[ai] id=%d  面积=%.0f px²  中心=(%.0f,%.0f)"
                          % (oid, area, (x1 + x2) / 2, (y1 + y2) / 2))

        self.pub_ids.publish(Int32MultiArray(data=ids))
        if n_ok == 0:
            rospy.logdebug_throttle(2, "[ai] 这一帧没有可用目标")


def main():
    rospy.init_node('ai_locate', anonymous=False)
    AiLocate()
    rospy.spin()


if __name__ == '__main__':
    main()

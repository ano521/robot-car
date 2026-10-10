#!/usr/bin/env python
import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from ultralytics import YOLO
from track.msg import DetectionInfo 

# 初始化ROS节点
rospy.init_node('tracker_node')

# 创建CvBridge对象
bridge = CvBridge()

# 加载模型
model = YOLO("/media/abot/999/abot/src/track/model/track.pt")

# 创建发布者，发布追踪结果图像
track_result_pub = rospy.Publisher('/track_result', Image, queue_size=10)

# 创建发布者，发布识别信息
detection_info_pub = rospy.Publisher('/detection_info', DetectionInfo, queue_size=10)

def image_callback(msg):
    try:
        # 将ROS图像消息转换为OpenCV格式
        frame = bridge.imgmsg_to_cv2(msg, "bgr8")
    except Exception as e:
        rospy.logerr(e)
        return

    # 运行追踪
    results = model.track(frame, persist=True)
    track_ids = results[0].boxes.id.int().cpu().tolist()
    bb = results[0].boxes
    boxes = results[0].boxes.xywh.cpu()
    # 获取追踪结果
    if results[0].boxes is not None:
        for box, track_id,m in zip(boxes, track_ids,bb):
        #for box in results[0].boxes:
            # 获取识别信息
            class_id = int(m.cls.item())
            confidence = float(m.conf.item())
            x, y, width, height = box

            # 创建识别信息对象
            detection_info = DetectionInfo()
            detection_info.track_id = track_id
            detection_info.class_id = class_id
            detection_info.confidence = confidence
            detection_info.x = x
            detection_info.y = y
            detection_info.width = width
            detection_info.height = height

            # 发布识别信息
            detection_info_pub.publish(detection_info)

    # 可视化追踪结果
    annotated_frame = results[0].plot()

    # 将OpenCV图像转换为ROS图像消息
    track_result_msg = bridge.cv2_to_imgmsg(annotated_frame, "bgr8")

    # 发布追踪结果图像
    track_result_pub.publish(track_result_msg)

# 订阅/usb_cam/image_raw话题
rospy.Subscriber('/usb_cam/image_raw', Image, image_callback)

# 保持节点运行
rospy.spin()

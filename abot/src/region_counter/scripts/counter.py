#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
from geometry_msgs.msg import Point
from region_counter.msg import CountResult  # 自定义消息类型

def is_inside_region(point, region_points):
    """
    判断点是否在指定区域内
    """
    return cv2.pointPolygonTest(np.array(region_points), point, False) >= 0

def image_callback(msg):
    global bridge, model, region_points, count_result_pub, count_image_pub

    # 将ROS图像消息转换为OpenCV图像
    im0 = bridge.imgmsg_to_cv2(msg, "bgr8")

    # 进行目标检测
    tracks = model.track(im0, persist=True)
    annotated_frame = tracks[0].plot()
    boxes = tracks[0].boxes.data
    count_class1 = 0
    count_class2 = 0
    center_points = []  # 存储中心点的列表

    for i, box in enumerate(boxes):
        x1, y1, x2, y2 = box[:4]
        x_center = (x1 + x2) / 2
        y_center = (y1 + y2) / 2
        center_point = (int(x_center), int(y_center))
        if is_inside_region(center_point, region_points):
            if box[-1] == 0:  # 类别1的标签 人
                count_class1 += 1
            elif box[-1] == 2:  # 类别2的标签 车
                count_class2 += 1
            # 将中心点添加到列表中
            center_points.append(Point(x=x_center, y=y_center, z=0))

    # 绘制区域
    region_mask = np.zeros_like(annotated_frame)
    cv2.fillPoly(region_mask, [np.array(region_points)], color=(255, 0, 0))
    alpha = 0.2  # 调整透明度
    annotated_frame = cv2.addWeighted(annotated_frame, 1 - alpha, region_mask, alpha, 0)

    # 添加文本
    text1 = "count_class1:%d" % count_class1
    text2 = "count_class2:%d" % count_class2
    cv2.putText(annotated_frame, text1, (20, 30), cv2.FONT_ITALIC, 1.0, (0, 255, 255), 2)
    cv2.putText(annotated_frame, text2, (20, 60), cv2.FONT_ITALIC, 1.0, (0, 255, 255), 2)

    # 发布处理后的图像
    count_image_msg = bridge.cv2_to_imgmsg(annotated_frame, "bgr8")
    count_image_pub.publish(count_image_msg)

    # 发布识别结果
    count_result_msg = CountResult()
    count_result_msg.count_class1 = count_class1
    count_result_msg.count_class2 = count_class2
    count_result_msg.center_points = center_points  # 填充中心点数组
    count_result_pub.publish(count_result_msg)

def main():
    global bridge, model, region_points, count_result_pub, count_image_pub

    # 初始化ROS节点
    rospy.init_node('region_counter_node', anonymous=True)

    # 创建CvBridge对象
    bridge = CvBridge()

    # 加载模型
    model = YOLO("/home/abot/abot/src/region_counter/model/counter.pt")

    # 定义区域点
    region_points = [(20, 20), (20, 400), (600, 400), (600, 20)]

    # 创建发布者
    count_image_pub = rospy.Publisher('/count_image', Image, queue_size=10)
    count_result_pub = rospy.Publisher('/count_result', CountResult, queue_size=10)

    # 订阅图像话题
    rospy.Subscriber('/usb_cam/image_raw', Image, image_callback)

    # 保持节点运行
    rospy.spin()

if __name__ == '__main__':
    main()

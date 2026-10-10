#!/usr/bin/env python

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from ultralytics import YOLO
from ultralytics.utils.plotting import Annotator, colors
from std_msgs.msg import String

class YOLONode:
    def __init__(self):
        rospy.init_node('sam_node', anonymous=True)
        
        # 加载模型
        self.model = YOLO("/home/abot/abot/src/sam/model/seg.pt")  # 分割模型
        self.names = self.model.model.names
        
        # 初始化cv_bridge
        self.bridge = CvBridge()
        
        # 订阅图像话题
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.image_callback)
        
        # 发布识别结果
        self.result_pub = rospy.Publisher("results", String, queue_size=10)
        
        # 发布带注释的图像
        self.image_pub = rospy.Publisher("annotated_image", Image, queue_size=10)

    def image_callback(self, data):
        try:
            # 将ROS图像消息转换为OpenCV图像
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        # 执行预测
        try:
            results = self.model.predict(cv_image)
        except Exception as e:
            rospy.logerr(f"预测过程中发生错误: {e}")
            return

        annotator = Annotator(cv_image, line_width=2)

        # 初始化clss为空列表
        clss = []

        if results[0].masks is not None:
            clss = results[0].boxes.cls.cpu().tolist()
            masks = results[0].masks.xy
            for mask, cls in zip(masks, clss):
                color = colors(int(cls), True)
                txt_color = annotator.get_txt_color(color)
                annotator.seg_bbox(mask=mask, mask_color=color, label=self.names[int(cls)], txt_color=txt_color)

        # 发布识别结果
        result_str = " ".join([self.names[int(cls)] for cls in clss])
        self.result_pub.publish(result_str)

        # 将带注释的图像转换为ROS图像消息并发布
        try:
            ros_image = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
            self.image_pub.publish(ros_image)
        except CvBridgeError as e:
            rospy.logerr(e)

        # # 显示带注释的图像
        # cv2.imshow("instance-segmentation", cv_image)
        # cv2.waitKey(1)

if __name__ == '__main__':
    try:
        node = YOLONode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()

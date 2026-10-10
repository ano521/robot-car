#!/usr/bin/env python
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
from yolov8_ros.msg import Detection  # 导入自定义消息类型

class YOLOv8ROSNode:
    def __init__(self):
        rospy.init_node('yolov8_ros_node', anonymous=True)
        self.bridge = CvBridge()
        self.model = YOLO('/home/abot/yolov8/yolov8n.pt')
        self.image_sub = rospy.Subscriber('/usb_cam/image_raw', Image, self.image_callback)
        self.detection_pub = rospy.Publisher('/yolov8_detections', Detection, queue_size=10)
        
        # 定义感兴趣的类别
        self.interested_classes = {"car","clock","person"}

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr(e)
            return

        # 对摄像头帧进行推理
        results = self.model(frame,conf=0.25)
        annotated_frame = results[0].plot()

        # 发布回归框信息
        for result in results:
            boxes = result.boxes.cpu().numpy()
            for box in boxes:
                x1, y1, x2, y2 = box.xyxy[0]
                cls = result.names[int(box.cls[0])]
                conf = box.conf[0]
                
                # 只处理感兴趣的类别
                if cls in self.interested_classes:
                    rospy.loginfo(f"Class: {cls}, Confidence: {conf:.2f}, Bbox: [{x1:.2f}, {y1:.2f}, {x2:.2f}, {y2:.2f}],Center:[{int((x2-x1)/2+x1)},{int((y2-y1)/2+y1)}]")

                    # 创建并发布检测消息
                    detection_msg = Detection()
                    detection_msg.class_name = cls
                    detection_msg.confidence = conf
                    detection_msg.x1 = x1
                    detection_msg.y1 = y1
                    detection_msg.x2 = x2
                    detection_msg.y2 = y2
                    self.detection_pub.publish(detection_msg)

        cv2.imshow("YOLOv8 Inference", annotated_frame)
        cv2.waitKey(1)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = YOLOv8ROSNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()

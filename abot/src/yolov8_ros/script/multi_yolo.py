#!/usr/bin/env python3
# 2026-10-28：shebang 由 python 改成 python3 —— 本文件用了 f-string（`rospy.loginfo(f"…")`），
# 只可能是 py3 代码。若车上的 `python` 指向 py2，rosrun 直接 SyntaxError 起不来。
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
        # 权重路径原本写死 /home/abot/yolov8/yolov8n.pt（车内路径，仓库里没带这个文件），
        # 文件不在时 ultralytics 直接抛 FileNotFoundError、节点秒退，所以留个参数口子：
        #   rosparam set /yolov8_ros/model_path <车上看得到 yolov8n.pt 的绝对路径>
        model_path = rospy.get_param('/yolov8_ros/model_path', '/home/abot/yolov8/yolov8n.pt')
        self.model = YOLO(model_path)
        # ★ 2026-10-28 修：原来订阅 '/usb_cam/image_raw'（相机 A / video0），
        #   但本包 usb_cam.launch 起的节点叫 usb_cam2、发的是 /usb_cam2/image_raw
        #   （相机 B / video2），RViz 的 Image 面板订的也是后者。
        #   结果：YOLO 永远收不到一帧 —— RViz 里有画面，但 /yolov8_detections
        #   一直是空的，grab.py / put.py 就永远等不到目标（看着像"卡住不动"）。
        #   grasp.sh 里没有任何节点发 /usb_cam/image_raw，只有 3-amcl_test.sh
        #   的 cam_track 会发。现在默认对齐到 usb_cam2，可用全局参数覆盖：
        #   rosparam set /yolov8_ros/image_topic /usb_cam/image_raw
        image_topic = rospy.get_param('/yolov8_ros/image_topic', '/usb_cam2/image_raw')
        self.image_sub = rospy.Subscriber(image_topic, Image, self.image_callback)
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

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import copy

import cv2 as cv
import numpy as np
import mediapipe as mp
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Header

from utils import CvFpsCalc


class Face_Dect:
    def __init__(self):
        image_topic = rospy.get_param('~camera_topic', '/usb_cam/image_raw')

        self.getImageStatus = False

        self.min_detection_confidence = 0.7
        self.plot_world_landmark = False


        self.mp_face_detection = mp.solutions.face_detection
        self.face_detection = self.mp_face_detection.FaceDetection(
            min_detection_confidence=self.min_detection_confidence,
        )

        self.color_image = Image()
        self.cvFpsCalc = CvFpsCalc(buffer_len=10)

        self.color_sub = rospy.Subscriber(image_topic, Image, self.image_callback, queue_size=1, buff_size=52428800)
        self.image_pub = rospy.Publisher('/wcxc/detection_image',  Image, queue_size=1)

        if self.plot_world_landmark:
            import matplotlib.pyplot as plt

            fig = plt.figure()
            r_ax = fig.add_subplot(121, projection="3d")
            l_ax = fig.add_subplot(122, projection="3d")
            fig.subplots_adjust(left=0.0, right=1, bottom=0, top=1)



        # if no image messages
        while (not self.getImageStatus) :
            rospy.loginfo("waiting for image.")
            rospy.sleep(2)

    def image_callback(self, image):
        display_fps = self.cvFpsCalc.get()
        self.getImageStatus = True
        self.color_image = np.frombuffer(image.data, dtype=np.uint8).reshape(image.height, image.width, -1)
        self.color_image = cv.cvtColor(self.color_image, cv.COLOR_BGR2RGB)

        debug_image = copy.deepcopy(self.color_image)

        results = self.face_detection.process(self.color_image)
        if results.detections is not None:
            for detection in results.detections:
                debug_image = self.draw_detection(debug_image, detection)

        cv.putText(debug_image, "FPS:" + str(display_fps), (10, 30), cv.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2, cv.LINE_AA)
        self.publish_image(debug_image, image.height, image.width)

        cv.imshow('MediaPipe Face Detection Demo', debug_image)
        key = cv.waitKey(1)


    def draw_detection(self, image, detection):
        image_width, image_height = image.shape[1], image.shape[0]

        # バウンディングボックス
        bbox = detection.location_data.relative_bounding_box
        bbox.xmin = int(bbox.xmin * image_width)
        bbox.ymin = int(bbox.ymin * image_height)
        bbox.width = int(bbox.width * image_width)
        bbox.height = int(bbox.height * image_height)

        cv.rectangle(image, (int(bbox.xmin), int(bbox.ymin)),
                     (int(bbox.xmin + bbox.width), int(bbox.ymin + bbox.height)),
                     (0, 255, 0), 2)

        # スコア・ラベルID
        cv.putText(
            image,
            str(detection.label_id[0]) + ":" + str(round(detection.score[0], 3)),
            (int(bbox.xmin), int(bbox.ymin) - 20), cv.FONT_HERSHEY_SIMPLEX, 1.0,
            (0, 255, 0), 2, cv.LINE_AA)

        # キーポイント：右目
        keypoint0 = detection.location_data.relative_keypoints[0]
        keypoint0.x = int(keypoint0.x * image_width)
        keypoint0.y = int(keypoint0.y * image_height)

        cv.circle(image, (int(keypoint0.x), int(keypoint0.y)), 5, (0, 255, 0), 2)

        # キーポイント：左目
        keypoint1 = detection.location_data.relative_keypoints[1]
        keypoint1.x = int(keypoint1.x * image_width)
        keypoint1.y = int(keypoint1.y * image_height)

        cv.circle(image, (int(keypoint1.x), int(keypoint1.y)), 5, (0, 255, 0), 2)

        # キーポイント：鼻
        keypoint2 = detection.location_data.relative_keypoints[2]
        keypoint2.x = int(keypoint2.x * image_width)
        keypoint2.y = int(keypoint2.y * image_height)

        cv.circle(image, (int(keypoint2.x), int(keypoint2.y)), 5, (0, 255, 0), 2)

        # キーポイント：口
        keypoint3 = detection.location_data.relative_keypoints[3]
        keypoint3.x = int(keypoint3.x * image_width)
        keypoint3.y = int(keypoint3.y * image_height)

        cv.circle(image, (int(keypoint3.x), int(keypoint3.y)), 5, (0, 255, 0), 2)

        # キーポイント：右耳
        keypoint4 = detection.location_data.relative_keypoints[4]
        keypoint4.x = int(keypoint4.x * image_width)
        keypoint4.y = int(keypoint4.y * image_height)

        cv.circle(image, (int(keypoint4.x), int(keypoint4.y)), 5, (0, 255, 0), 2)

        # キーポイント：左耳
        keypoint5 = detection.location_data.relative_keypoints[5]
        keypoint5.x = int(keypoint5.x * image_width)
        keypoint5.y = int(keypoint5.y * image_height)

        cv.circle(image, (int(keypoint5.x), int(keypoint5.y)), 5, (0, 255, 0), 2)

        return image
    def publish_image(self, imgdata, height, width):
        image_temp = Image()
        header = Header(stamp=rospy.Time.now())
        header.frame_id = "camera"
        image_temp.height = height
        image_temp.width = width
        image_temp.encoding = 'bgr8'
        image_temp.data = np.array(imgdata).tobytes()
        image_temp.header = header
        image_temp.step = width * 3
        self.image_pub.publish(image_temp)


def main():
    rospy.init_node('abot_mediapipe', anonymous=True)
    face_dect = Face_Dect()
    rospy.spin()

if __name__ == '__main__':
    main()

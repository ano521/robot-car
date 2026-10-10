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

class Objectron:
    def __init__(self):
        image_topic = rospy.get_param('~camera_topic', '/usb_cam/image_raw')

        self.getImageStatus = False

        self.static_image_mode = True
        self.max_num_objects = 5
        self.min_detection_confidence = 0.5
        self.min_tracking_confidence = 0.99
        self.model_name = 'Cup' # {'Shoe', 'Chair', 'Cup', 'Camera'}

        self.mp_objectron = mp.solutions.objectron
        objectron = self.mp_objectron.Objectron(
            static_image_mode=self.static_image_mode,
            max_num_objects=self.max_num_objects,
            min_detection_confidence=self.min_detection_confidence,
            min_tracking_confidence=self.min_tracking_confidence,
            model_name=self.model_name,
        )

        self.p_drawing = mp.solutions.drawing_utils

        self.color_image = Image()
        self.cvFpsCalc = CvFpsCalc(buffer_len=10)

        self.color_sub = rospy.Subscriber(image_topic, Image, self.image_callback, queue_size=1, buff_size=52428800)
        self.image_pub = rospy.Publisher('/wcxc/detection_image',  Image, queue_size=1)

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

        results = objectron.process(self.color_image)

        # 描画 ################################################################
        if results.detected_objects is not None:
            for detected_object in results.detected_objects:
                mp_drawing.draw_landmarks(debug_image,
                                          detected_object.landmarks_2d,
                                          mp_objectron.BOX_CONNECTIONS)
                mp_drawing.draw_axis(debug_image, detected_object.rotation,
                                     detected_object.translation)

                # キーポイント確認用
                draw_landmarks(debug_image, detected_object.landmarks_2d)


        cv.putText(debug_image, "FPS:" + str(display_fps), (10, 30), cv.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2, cv.LINE_AA)
        self.publish_image(debug_image, image.height, image.width)

        cv.imshow('MediaPipe Face Detection Demo', debug_image)
        key = cv.waitKey(1)




    def draw_landmarks(image, landmarks):
        image_width, image_height = image.shape[1], image.shape[0]

        landmark_point = []

        for index, landmark in enumerate(landmarks.landmark):
            landmark_x = min(int(landmark.x * image_width), image_width - 1)
            landmark_y = min(int(landmark.y * image_height), image_height - 1)
            landmark_point.append([(landmark_x, landmark_y)])

            if index == 0:  # 重心
                cv.circle(image, (landmark_x, landmark_y), 5, (0, 255, 0), 2)
            if index == 1:  #
                cv.circle(image, (landmark_x, landmark_y), 5, (0, 255, 0), 2)
            if index == 2:  #
                cv.circle(image, (landmark_x, landmark_y), 5, (0, 255, 0), 2)
            if index == 3:  #
                cv.circle(image, (landmark_x, landmark_y), 5, (0, 255, 0), 2)
            if index == 4:  #
                cv.circle(image, (landmark_x, landmark_y), 5, (0, 255, 0), 2)
            if index == 5:  #
                cv.circle(image, (landmark_x, landmark_y), 5, (0, 255, 0), 2)
            if index == 6:  #
                cv.circle(image, (landmark_x, landmark_y), 5, (0, 255, 0), 2)
            if index == 7:  #
                cv.circle(image, (landmark_x, landmark_y), 5, (0, 255, 0), 2)
            if index == 8:  #
                cv.circle(image, (landmark_x, landmark_y), 5, (0, 255, 0), 2)

        return image


    def draw_bounding_rect(use_brect, image, brect):
        if use_brect:
            # 外接矩形
            cv.rectangle(image, (brect[0], brect[1]), (brect[2], brect[3]),
                         (0, 255, 0), 2)

        return image

def main():
    rospy.init_node('abot_mediapipe', anonymous=True)
    objectron = Objectron()
    rospy.spin()


if __name__ == '__main__':
    main()

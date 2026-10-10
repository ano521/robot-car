#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import copy

import cv2 as cv
import numpy as np
import mediapipe as mp
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Header
import matplotlib.pyplot as plt

from utils import CvFpsCalc


class Pose:
    def __init__(self):
        image_topic = rospy.get_param('~camera_topic', '/usb_cam/image_raw')

        self.getImageStatus = False
        # upper_body_only = args.upper_body_only
        self.model_complexity = 1
        self.min_detection_confidence = 0.5
        self.min_tracking_confidence = 0.5
        self.enable_segmentation = False
        self.segmentation_score_th = 0.5

        self.use_brect = False
        self.plot_world_landmark = False

        self.mp_pose = mp.solutions.pose
        self.pose = self.mp_pose.Pose(
            # upper_body_only=upper_body_only,
            model_complexity=self.model_complexity,
            min_detection_confidence=self.min_detection_confidence,
            min_tracking_confidence=self.min_tracking_confidence,
        )


        self.color_image = Image()
        self.cvFpsCalc = CvFpsCalc(buffer_len=10)

        if self.plot_world_landmark:
            fig = plt.figure()
            self.ax = fig.add_subplot(111, projection="3d")
            fig.subplots_adjust(left=0.0, right=1, bottom=0, top=1)


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
        results = self.pose.process(self.color_image)

        # 描画 ################################################################
        if self.enable_segmentation and results.segmentation_mask is not None:
            # セグメンテーション
            mask = np.stack((results.segmentation_mask, ) * 3,
                            axis=-1) > self.segmentation_score_th
            bg_resize_image = np.zeros(self.color_image.shape, dtype=np.uint8)
            bg_resize_image[:] = (0, 255, 0)
            debug_image = np.where(mask, debug_image, bg_resize_image)


        if results.pose_landmarks is not None:
            # 外接矩形の計算
            brect = self.calc_bounding_rect(debug_image, results.pose_landmarks)
            # 描画
            debug_image = self.draw_landmarks(
                debug_image,
                results.pose_landmarks,
                # upper_body_only,
            )
            debug_image = self.draw_bounding_rect(self.use_brect, debug_image, brect)

         # World座標プロット ###################################################
        if self.plot_world_landmark:
            if results.pose_world_landmarks is not None:
                self.plot_world_landmarks(
                    plt,
                    self.ax,
                    results.pose_world_landmarks,
                )

        if self.enable_segmentation and results.segmentation_mask is not None:
            fps_color = (255, 255, 255)
        else:
            fps_color = (0, 255, 0)
        cv.putText(debug_image, "FPS:" + str(display_fps), (10, 30),
                   cv.FONT_HERSHEY_SIMPLEX, 1.0, fps_color, 2, cv.LINE_AA)

        self.publish_image(debug_image, image.height, image.width)

        cv.imshow('MediaPipe Pose Demo', debug_image)
        key = cv.waitKey(1)





    def calc_bounding_rect(self, image, landmarks):
        image_width, image_height = image.shape[1], image.shape[0]

        landmark_array = np.empty((0, 2), int)

        for _, landmark in enumerate(landmarks.landmark):
            landmark_x = min(int(landmark.x * image_width), image_width - 1)
            landmark_y = min(int(landmark.y * image_height), image_height - 1)

            landmark_point = [np.array((landmark_x, landmark_y))]

            landmark_array = np.append(landmark_array, landmark_point, axis=0)

        x, y, w, h = cv.boundingRect(landmark_array)

        return [x, y, x + w, y + h]


    def draw_landmarks(
        self,
        image,
        landmarks,
        # upper_body_only,
        visibility_th=0.5,
    ):
        image_width, image_height = image.shape[1], image.shape[0]

        landmark_point = []

        for index, landmark in enumerate(landmarks.landmark):
            landmark_x = min(int(landmark.x * image_width), image_width - 1)
            landmark_y = min(int(landmark.y * image_height), image_height - 1)
            landmark_z = landmark.z
            landmark_point.append([landmark.visibility, (landmark_x, landmark_y)])

            if landmark.visibility < visibility_th:
                continue

            if index == 0:  # 鼻
                cv.circle(image, (landmark_x, landmark_y), 5, (0, 255, 0), 2)
            if index == 1:  # 右目：目頭
                cv.circle(image, (landmark_x, landmark_y), 5, (0, 255, 0), 2)
            if index == 2:  # 右目：瞳
                cv.circle(image, (landmark_x, landmark_y), 5, (0, 255, 0), 2)
            if index == 3:  # 右目：目尻
                cv.circle(image, (landmark_x, landmark_y), 5, (0, 255, 0), 2)
            if index == 4:  # 左目：目頭
                cv.circle(image, (landmark_x, landmark_y), 5, (0, 255, 0), 2)
            if index == 5:  # 左目：瞳
                cv.circle(image, (landmark_x, landmark_y), 5, (0, 255, 0), 2)
            if index == 6:  # 左目：目尻
                cv.circle(image, (landmark_x, landmark_y), 5, (0, 255, 0), 2)
            if index == 7:  # 右耳
                cv.circle(image, (landmark_x, landmark_y), 5, (0, 255, 0), 2)
            if index == 8:  # 左耳
                cv.circle(image, (landmark_x, landmark_y), 5, (0, 255, 0), 2)
            if index == 9:  # 口：左端
                cv.circle(image, (landmark_x, landmark_y), 5, (0, 255, 0), 2)
            if index == 10:  # 口：左端
                cv.circle(image, (landmark_x, landmark_y), 5, (0, 255, 0), 2)
            if index == 11:  # 右肩
                cv.circle(image, (landmark_x, landmark_y), 5, (0, 255, 0), 2)
            if index == 12:  # 左肩
                cv.circle(image, (landmark_x, landmark_y), 5, (0, 255, 0), 2)
            if index == 13:  # 右肘
                cv.circle(image, (landmark_x, landmark_y), 5, (0, 255, 0), 2)
            if index == 14:  # 左肘
                cv.circle(image, (landmark_x, landmark_y), 5, (0, 255, 0), 2)
            if index == 15:  # 右手首
                cv.circle(image, (landmark_x, landmark_y), 5, (0, 255, 0), 2)
            if index == 16:  # 左手首
                cv.circle(image, (landmark_x, landmark_y), 5, (0, 255, 0), 2)
            if index == 17:  # 右手1(外側端)
                cv.circle(image, (landmark_x, landmark_y), 5, (0, 255, 0), 2)
            if index == 18:  # 左手1(外側端)
                cv.circle(image, (landmark_x, landmark_y), 5, (0, 255, 0), 2)
            if index == 19:  # 右手2(先端)
                cv.circle(image, (landmark_x, landmark_y), 5, (0, 255, 0), 2)
            if index == 20:  # 左手2(先端)
                cv.circle(image, (landmark_x, landmark_y), 5, (0, 255, 0), 2)
            if index == 21:  # 右手3(内側端)
                cv.circle(image, (landmark_x, landmark_y), 5, (0, 255, 0), 2)
            if index == 22:  # 左手3(内側端)
                cv.circle(image, (landmark_x, landmark_y), 5, (0, 255, 0), 2)
            if index == 23:  # 腰(右側)
                cv.circle(image, (landmark_x, landmark_y), 5, (0, 255, 0), 2)
            if index == 24:  # 腰(左側)
                cv.circle(image, (landmark_x, landmark_y), 5, (0, 255, 0), 2)
            if index == 25:  # 右ひざ
                cv.circle(image, (landmark_x, landmark_y), 5, (0, 255, 0), 2)
            if index == 26:  # 左ひざ
                cv.circle(image, (landmark_x, landmark_y), 5, (0, 255, 0), 2)
            if index == 27:  # 右足首
                cv.circle(image, (landmark_x, landmark_y), 5, (0, 255, 0), 2)
            if index == 28:  # 左足首
                cv.circle(image, (landmark_x, landmark_y), 5, (0, 255, 0), 2)
            if index == 29:  # 右かかと
                cv.circle(image, (landmark_x, landmark_y), 5, (0, 255, 0), 2)
            if index == 30:  # 左かかと
                cv.circle(image, (landmark_x, landmark_y), 5, (0, 255, 0), 2)
            if index == 31:  # 右つま先
                cv.circle(image, (landmark_x, landmark_y), 5, (0, 255, 0), 2)
            if index == 32:  # 左つま先
                cv.circle(image, (landmark_x, landmark_y), 5, (0, 255, 0), 2)

            # if not upper_body_only:
            if True:
                cv.putText(image, "z:" + str(round(landmark_z, 3)),
                           (landmark_x - 10, landmark_y - 10),
                           cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1,
                           cv.LINE_AA)

        if len(landmark_point) > 0:
            # 右目
            if landmark_point[1][0] > visibility_th and landmark_point[2][
                    0] > visibility_th:
                cv.line(image, landmark_point[1][1], landmark_point[2][1],
                        (0, 255, 0), 2)
            if landmark_point[2][0] > visibility_th and landmark_point[3][
                    0] > visibility_th:
                cv.line(image, landmark_point[2][1], landmark_point[3][1],
                        (0, 255, 0), 2)

            # 左目
            if landmark_point[4][0] > visibility_th and landmark_point[5][
                    0] > visibility_th:
                cv.line(image, landmark_point[4][1], landmark_point[5][1],
                        (0, 255, 0), 2)
            if landmark_point[5][0] > visibility_th and landmark_point[6][
                    0] > visibility_th:
                cv.line(image, landmark_point[5][1], landmark_point[6][1],
                        (0, 255, 0), 2)

            # 口
            if landmark_point[9][0] > visibility_th and landmark_point[10][
                    0] > visibility_th:
                cv.line(image, landmark_point[9][1], landmark_point[10][1],
                        (0, 255, 0), 2)

            # 肩
            if landmark_point[11][0] > visibility_th and landmark_point[12][
                    0] > visibility_th:
                cv.line(image, landmark_point[11][1], landmark_point[12][1],
                        (0, 255, 0), 2)

            # 右腕
            if landmark_point[11][0] > visibility_th and landmark_point[13][
                    0] > visibility_th:
                cv.line(image, landmark_point[11][1], landmark_point[13][1],
                        (0, 255, 0), 2)
            if landmark_point[13][0] > visibility_th and landmark_point[15][
                    0] > visibility_th:
                cv.line(image, landmark_point[13][1], landmark_point[15][1],
                        (0, 255, 0), 2)

            # 左腕
            if landmark_point[12][0] > visibility_th and landmark_point[14][
                    0] > visibility_th:
                cv.line(image, landmark_point[12][1], landmark_point[14][1],
                        (0, 255, 0), 2)
            if landmark_point[14][0] > visibility_th and landmark_point[16][
                    0] > visibility_th:
                cv.line(image, landmark_point[14][1], landmark_point[16][1],
                        (0, 255, 0), 2)

            # 右手
            if landmark_point[15][0] > visibility_th and landmark_point[17][
                    0] > visibility_th:
                cv.line(image, landmark_point[15][1], landmark_point[17][1],
                        (0, 255, 0), 2)
            if landmark_point[17][0] > visibility_th and landmark_point[19][
                    0] > visibility_th:
                cv.line(image, landmark_point[17][1], landmark_point[19][1],
                        (0, 255, 0), 2)
            if landmark_point[19][0] > visibility_th and landmark_point[21][
                    0] > visibility_th:
                cv.line(image, landmark_point[19][1], landmark_point[21][1],
                        (0, 255, 0), 2)
            if landmark_point[21][0] > visibility_th and landmark_point[15][
                    0] > visibility_th:
                cv.line(image, landmark_point[21][1], landmark_point[15][1],
                        (0, 255, 0), 2)

            # 左手
            if landmark_point[16][0] > visibility_th and landmark_point[18][
                    0] > visibility_th:
                cv.line(image, landmark_point[16][1], landmark_point[18][1],
                        (0, 255, 0), 2)
            if landmark_point[18][0] > visibility_th and landmark_point[20][
                    0] > visibility_th:
                cv.line(image, landmark_point[18][1], landmark_point[20][1],
                        (0, 255, 0), 2)
            if landmark_point[20][0] > visibility_th and landmark_point[22][
                    0] > visibility_th:
                cv.line(image, landmark_point[20][1], landmark_point[22][1],
                        (0, 255, 0), 2)
            if landmark_point[22][0] > visibility_th and landmark_point[16][
                    0] > visibility_th:
                cv.line(image, landmark_point[22][1], landmark_point[16][1],
                        (0, 255, 0), 2)

            # 胴体
            if landmark_point[11][0] > visibility_th and landmark_point[23][
                    0] > visibility_th:
                cv.line(image, landmark_point[11][1], landmark_point[23][1],
                        (0, 255, 0), 2)
            if landmark_point[12][0] > visibility_th and landmark_point[24][
                    0] > visibility_th:
                cv.line(image, landmark_point[12][1], landmark_point[24][1],
                        (0, 255, 0), 2)
            if landmark_point[23][0] > visibility_th and landmark_point[24][
                    0] > visibility_th:
                cv.line(image, landmark_point[23][1], landmark_point[24][1],
                        (0, 255, 0), 2)

            if len(landmark_point) > 25:
                # 右足
                if landmark_point[23][0] > visibility_th and landmark_point[25][
                        0] > visibility_th:
                    cv.line(image, landmark_point[23][1], landmark_point[25][1],
                            (0, 255, 0), 2)
                if landmark_point[25][0] > visibility_th and landmark_point[27][
                        0] > visibility_th:
                    cv.line(image, landmark_point[25][1], landmark_point[27][1],
                            (0, 255, 0), 2)
                if landmark_point[27][0] > visibility_th and landmark_point[29][
                        0] > visibility_th:
                    cv.line(image, landmark_point[27][1], landmark_point[29][1],
                            (0, 255, 0), 2)
                if landmark_point[29][0] > visibility_th and landmark_point[31][
                        0] > visibility_th:
                    cv.line(image, landmark_point[29][1], landmark_point[31][1],
                            (0, 255, 0), 2)

                # 左足
                if landmark_point[24][0] > visibility_th and landmark_point[26][
                        0] > visibility_th:
                    cv.line(image, landmark_point[24][1], landmark_point[26][1],
                            (0, 255, 0), 2)
                if landmark_point[26][0] > visibility_th and landmark_point[28][
                        0] > visibility_th:
                    cv.line(image, landmark_point[26][1], landmark_point[28][1],
                            (0, 255, 0), 2)
                if landmark_point[28][0] > visibility_th and landmark_point[30][
                        0] > visibility_th:
                    cv.line(image, landmark_point[28][1], landmark_point[30][1],
                            (0, 255, 0), 2)
                if landmark_point[30][0] > visibility_th and landmark_point[32][
                        0] > visibility_th:
                    cv.line(image, landmark_point[30][1], landmark_point[32][1],
                            (0, 255, 0), 2)
        return image


    def plot_world_landmarks(
        self,
        plt,
        ax,
        landmarks,
        visibility_th=0.5,
    ):
        landmark_point = []

        for index, landmark in enumerate(landmarks.landmark):
            landmark_point.append(
                [landmark.visibility, (landmark.x, landmark.y, landmark.z)])

        face_index_list = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10]
        right_arm_index_list = [11, 13, 15, 17, 19, 21]
        left_arm_index_list = [12, 14, 16, 18, 20, 22]
        right_body_side_index_list = [11, 23, 25, 27, 29, 31]
        left_body_side_index_list = [12, 24, 26, 28, 30, 32]
        shoulder_index_list = [11, 12]
        waist_index_list = [23, 24]

        # 顔
        face_x, face_y, face_z = [], [], []
        for index in face_index_list:
            point = landmark_point[index][1]
            face_x.append(point[0])
            face_y.append(point[2])
            face_z.append(point[1] * (-1))

        # 右腕
        right_arm_x, right_arm_y, right_arm_z = [], [], []
        for index in right_arm_index_list:
            point = landmark_point[index][1]
            right_arm_x.append(point[0])
            right_arm_y.append(point[2])
            right_arm_z.append(point[1] * (-1))

        # 左腕
        left_arm_x, left_arm_y, left_arm_z = [], [], []
        for index in left_arm_index_list:
            point = landmark_point[index][1]
            left_arm_x.append(point[0])
            left_arm_y.append(point[2])
            left_arm_z.append(point[1] * (-1))

        # 右半身
        right_body_side_x, right_body_side_y, right_body_side_z = [], [], []
        for index in right_body_side_index_list:
            point = landmark_point[index][1]
            right_body_side_x.append(point[0])
            right_body_side_y.append(point[2])
            right_body_side_z.append(point[1] * (-1))

        # 左半身
        left_body_side_x, left_body_side_y, left_body_side_z = [], [], []
        for index in left_body_side_index_list:
            point = landmark_point[index][1]
            left_body_side_x.append(point[0])
            left_body_side_y.append(point[2])
            left_body_side_z.append(point[1] * (-1))

        # 肩
        shoulder_x, shoulder_y, shoulder_z = [], [], []
        for index in shoulder_index_list:
            point = landmark_point[index][1]
            shoulder_x.append(point[0])
            shoulder_y.append(point[2])
            shoulder_z.append(point[1] * (-1))

        # 腰
        waist_x, waist_y, waist_z = [], [], []
        for index in waist_index_list:
            point = landmark_point[index][1]
            waist_x.append(point[0])
            waist_y.append(point[2])
            waist_z.append(point[1] * (-1))
                
        ax.cla()
        ax.set_xlim3d(-1, 1)
        ax.set_ylim3d(-1, 1)
        ax.set_zlim3d(-1, 1)

        ax.scatter(face_x, face_y, face_z)
        ax.plot(right_arm_x, right_arm_y, right_arm_z)
        ax.plot(left_arm_x, left_arm_y, left_arm_z)
        ax.plot(right_body_side_x, right_body_side_y, right_body_side_z)
        ax.plot(left_body_side_x, left_body_side_y, left_body_side_z)
        ax.plot(shoulder_x, shoulder_y, shoulder_z)
        ax.plot(waist_x, waist_y, waist_z)
        
        plt.pause(.001)

        return


    def draw_bounding_rect(self, use_brect, image, brect):
        if use_brect:
            # 外接矩形
            cv.rectangle(image, (brect[0], brect[1]), (brect[2], brect[3]),
                         (0, 255, 0), 2)

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
    pose = Pose()
    rospy.spin()


if __name__ == '__main__':
    main()

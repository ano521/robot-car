#!/usr/bin/env python3
# -*- coding: utf-8 -*-
'''
Copyright (c) [Zachary]
本代码受版权法保护，未经授权禁止任何形式的复制、分发、修改等使用行为。
Author:Zachary
company:WCXC
'''

from vl_locate.srv import GetObject,GetObjectResponse
import rospy
import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont
import time
from sensor_msgs.msg import Image as ROSImage
from std_msgs.msg import String
import json
import base64
import sys
import os
from geometry_msgs.msg import PoseStamped
from dashscope import MultiModalConversation

# 系统提示词（修正中文标点为英文标点）
SYSTEM_PROMPT = """
检测图像中的目标，并以坐标的形式返回其位置。
例如检测图像中的绿色方块，输出格式应为 {"label": "绿色方块", "bbox_2d": [x1, y1, x2, y2]}。
只回复json本身即可，不要回复其它内容

我现在的指令是：
"""

class VLM:
    def __init__(self):
        self.sub = rospy.Subscriber('/usb_cam/image_raw', ROSImage, self.top_view_shot)
        self.image_path = '/media/abot/999/abot/src/vl_locate/image/vlm_now.jpg'
        self.font = ImageFont.truetype("/media/abot/999/abot/src/vl_locate/config/SimHei.ttf", 32)
        self.dashscope_api_key = os.environ.get('DASHSCOPE_API_KEY')
        self.dashscope_model = os.environ.get('DASHSCOPE_MODEL')
        s = rospy.Service('vlm_detection', GetObject, self.handle_vlm_detection)

    def encode_image(self, image_path):
        with open(image_path, "rb") as image_file:
            return base64.b64encode(image_file.read()).decode('utf-8')

    def imgmsg_to_cv2(self, img_msg):
        dtype = np.dtype("uint8")
        dtype = dtype.newbyteorder('>' if img_msg.is_bigendian else '<')
        image_opencv = np.ndarray(shape=(img_msg.height, img_msg.width, 3),
                                 dtype=dtype, buffer=img_msg.data)
        if img_msg.is_bigendian != sys.byteorder == 'little':
            image_opencv = image_opencv.byteswap().newbyteorder()
        if img_msg.encoding == "rgb8":
            image_opencv = cv2.cvtColor(image_opencv, cv2.COLOR_RGB2BGR)
        elif img_msg.encoding == "mono8":
            image_opencv = cv2.cvtColor(image_opencv, cv2.COLOR_GRAY2BGR)
        elif img_msg.encoding != "bgr8":
            rospy.logerr("Unsupported encoding: %s", img_msg.encoding)
            return None
        return image_opencv

    def top_view_shot(self, image_msg):
        self.img_bgr = self.imgmsg_to_cv2(image_msg)

    def yi_vision_api(self, prompt='检测图中红色方块'):
        if not self.dashscope_api_key or not self.dashscope_model:
            rospy.logerr(
                'VLM is not configured: set DASHSCOPE_API_KEY and DASHSCOPE_MODEL '
                'before starting vl_locate.'
            )
            return None
        if not hasattr(self, 'img_bgr') or self.img_bgr is None:
            rospy.logerr('No camera image has been received yet.')
            return None

        cv2.imwrite(self.image_path, self.img_bgr)
        base64_image = self.encode_image(self.image_path)
        messages = [
            {
                "role": "user",
                "content": [
                    {"image": self.image_path},
                    {"text": SYSTEM_PROMPT + prompt}
                ]
            }
        ]
        
        response = MultiModalConversation.call(
            api_key=self.dashscope_api_key,
            model=self.dashscope_model,
            messages=messages
        )
        
        content = response["output"]["choices"][0]["message"].content[0]["text"]
        
        # 去除Markdown代码块标记（如果存在）
        if content.startswith("```json") and content.endswith("```"):
            content = content[7:-3].strip()
        
        try:
            result = json.loads(content)
            # 如果返回的是列表，取第一个元素
            if isinstance(result, list):
                result = result[0]
            rospy.loginfo('大模型调用成功！')
            return result
        except json.JSONDecodeError as e:
            rospy.logerr(f"JSON解析失败: {e}")
            rospy.logerr(f"返回内容: {content}")
            return None

    def post_processing_viz(self, result, img_path):
        img_bgr = cv2.imread(img_path)
        img_h, img_w = img_bgr.shape[:2]
        
        START_NAME = result['label']
        bbox_2d = result['bbox_2d']
        rospy.loginfo(f"检测到目标: {START_NAME}, 坐标: {bbox_2d}")
        
        x_min, y_min, x_max, y_max = map(int, bbox_2d)
        center_x = int((x_min + x_max) / 2)
        center_y = int((y_min + y_max) / 2)
        
        # 可视化
        img_bgr = cv2.rectangle(img_bgr, (x_min, y_min), (x_max, y_max), (0,0,255), 3)
        img_bgr = cv2.circle(img_bgr, (center_x, center_y), 6, (0,0,255), -1)
        
        # 写中文标签
        img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
        img_pil = Image.fromarray(img_rgb)
        draw = ImageDraw.Draw(img_pil)
        draw.text((x_min, y_min-32), START_NAME, font=self.font, fill=(255,0,0))
        img_bgr = cv2.cvtColor(np.array(img_pil), cv2.COLOR_RGB2BGR)
        
        cv2.imwrite('/media/abot/999/abot/src/vl_locate/image/vlm_now.jpg', img_bgr)
        return center_x, center_y

    def vlm_detection(self):
        result = self.yi_vision_api('检测图中的{}'.format(self.object_name))
        if not isinstance(result, dict):
            rospy.logerr("返回结果格式错误！")
            return None
        
        X_CENTER, Y_CENTER = self.post_processing_viz(result, self.image_path)
        
        # 发布位姿信息
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = "camera_link"
        pose_msg.pose.position.x = X_CENTER
        pose_msg.pose.position.y = Y_CENTER
        pose_msg.pose.orientation.w = 1.0
        pose_pub.publish(pose_msg)
        return (X_CENTER, Y_CENTER)

    def handle_vlm_detection(self,req):
        self.object_name=str(req.object_name)
        result = self.vlm_detection()
        if result:
            return GetObjectResponse(success=True, message=f"检测到目标坐标：{result}")
        else:
            return GetObjectResponse(success=False, message="检测失败")

def main():
    global vlm_grasp, pose_pub
    rospy.init_node('vlm_grasp', anonymous=True)
    vlm_grasp = VLM()
    pose_pub = rospy.Publisher('/Pose', PoseStamped, queue_size=10)
    rospy.loginfo("VL检测服务已启动，等待请求...")
    rospy.spin()

if __name__ == '__main__':
    main()

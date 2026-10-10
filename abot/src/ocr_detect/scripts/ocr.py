#!/usr/bin/env python
import rospy
from cnocr import CnOcr
from PIL import Image, ImageDraw, ImageFont
import cv2
import numpy as np
import os

def get_bbox(array):
    "将结果中的position信息的四个点的坐标信息转换"
    x1 = array[0][0]
    y1 = array[0][1]
    pt1 = (int(x1), int(y1))
    x2 = array[2][0]
    y2 = array[2][1]
    pt2 = (int(x2), int(y2))
    return pt1, pt2

def dealImg(img):
    b, g, r = cv2.split(img)
    img_rgb = cv2.merge([r, g, b])
    return img_rgb

def create_blank_img(img_w, img_h):
    # 将图像宽度调整为原来的1.5倍
    img_w = int(img_w * 1.5)
    blank_img = np.ones(shape=[img_h, img_w, 3], dtype=np.uint8) * 255
    # blank_img[:, img_w - 1:] = 0
    blank_img = Image.fromarray(blank_img)
    return blank_img

def Draw_OCRResult(blank_img, pt1, pt2, text):
    # 确保 blank_img 是 PIL.Image 对象
    if isinstance(blank_img, np.ndarray):
        blank_img = Image.fromarray(blank_img)
    
    draw = ImageDraw.Draw(blank_img)
    draw.rectangle([pt1, pt2], outline=(255, 255, 0), width=3)
    fontStyle = ImageFont.truetype("/media/abot/999/abot/src/ocr_detect/ChineseFonts/simsun.ttc", size=21, encoding="utf-8")
    (x, y) = pt1
    draw.text((x+5, y+5), text=text, fill=(0, 0, 0), font=fontStyle)
    blank_img = np.asarray(blank_img)
    return blank_img

def ocr_detect():
    rospy.init_node('ocr_detect', anonymous=True)
    img_path = rospy.get_param('image_path', "/media/abot/999/abot/src/ocr_detect/image/11.jpg")
    
    while not rospy.is_shutdown():
        im = cv2.imread(img_path)
        if im is None:
            rospy.logwarn("Failed to load image: %s", img_path)
            rospy.sleep(1)
            continue
        
        img_h, img_w, _ = im.shape
        blank_img = create_blank_img(img_w, img_h)
        # 所有参数都使用默认值
        ocr = CnOcr()
        result = ocr.ocr(img_path)
        # print(result)
        for temp in result:
            print(temp["text"])
            # print(temp["score"])
            pt1, pt2 = get_bbox(temp["position"])
            blank_img = Draw_OCRResult(blank_img, pt1, pt2, temp["text"])
        images = np.concatenate((im, blank_img), axis=1)
        
        # 显示图片
        cv2.imshow('OCR Result', images)
        cv2.waitKey(0)  # 等待按键
        
        # 保存图片
        # 提取文件名部分
        base_name = os.path.basename(img_path)
        file_name, _ = os.path.splitext(base_name)
        save_path = f'/media/abot/999/abot/src/ocr_detect/result/OCR_result_{file_name}.jpg'
        cv2.imwrite(save_path, images)
        
        # 等待新的图片路径更新
        rospy.loginfo("Waiting for new image path...")
        while not rospy.is_shutdown():
            new_img_path = rospy.get_param('image_path', img_path)
            if new_img_path != img_path:
                img_path = new_img_path
                rospy.loginfo("Updated image path to: %s", img_path)
                break
            rospy.sleep(1)

if __name__ == '__main__':
    try:
        ocr_detect()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()  # 关闭所有窗口

import cv2
import numpy as np
from ultralytics import YOLO


def is_inside_region(point, region_points):
    """
    判断点是否在指定区域内
    """
    return cv2.pointPolygonTest(np.array(region_points), point, False) >= 0

def detect():

    model = YOLO("/home/abot/yolov8/yolov8n.pt")
    cv2.namedWindow('region couter', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('region couter', 640, 480)  # 设置宽高
    # cap = cv2.VideoCapture('ultralytics/assets/a3.mp4')
    cap = cv2.VideoCapture(0)
    out_video = cv2.VideoWriter('output.avi', cv2.VideoWriter_fourcc(*'XVID'), 30, (2560, 720))

    region_points = [(20, 20), (20, 400), (600, 400), (600, 20)]
    region_points_np = np.array(region_points)

    assert cap.isOpened(), "Error reading video file"
    while cap.isOpened():
        success, im0 = cap.read()
        if not success:
            print("Video frame is empty or video processing has been successfully completed.")
            break
        tracks = model.track(im0, persist=True)
        annotated_frame = tracks[0].plot()
        boxes = tracks[0].boxes.data
        count_class1 = 0
        count_class2 = 0
        for i, box in enumerate(boxes):
            x1, y1, x2, y2 = box[:4]
            x_center = (x1 + x2) / 2
            y_center = (y1 + y2) / 2
            center_point = (int(x_center), int(y_center))
            if is_inside_region(center_point, region_points):
                if box[-1] == 0:  # 类别1的标签 人
                    count_class1 += 1
                elif  box[-1] == 2:  # 类别2的标签 车
                    count_class2 += 1
        # cv2.polylines(annotated_frame, [region_points_np], isClosed=True, color=(255, 0, 0), thickness=2)
        region_mask = np.zeros_like(annotated_frame)
        cv2.fillPoly(region_mask, [region_points_np], color=(255, 0, 0))
        # 使用透明度将填充后的区域与原始帧混合
        alpha = 0.2  # 调整透明度
        annotated_frame = cv2.addWeighted(annotated_frame, 1 - alpha, region_mask, alpha, 0)

        text1 = "count_class1:%d" % count_class1
        text2 = "count_class2:%d" % count_class2
        cv2.putText(annotated_frame, text1, (20, 30), cv2.FONT_ITALIC, 1.0, (0, 255, 255), 2)
        cv2.putText(annotated_frame, text2, (20, 60), cv2.FONT_ITALIC, 1.0, (0, 255, 255), 2)
        print("Number of objects in class 1:", count_class1)
        print("Number of objects in class 2:", count_class2)

        cv2.imshow("region couter", annotated_frame)
        out_video.write(annotated_frame)
        if cv2.waitKey(1) == ord('q'):
            break
    out_video.release()
    cap.release()
    cv2.destroyAllWindows()
if __name__ == '__main__':
    detect()

import cv2
import numpy as np

def map_server_to_cv2(data):
    # print(data.info.height, data.info.width)
    data = np.reshape(data.data, (data.info.height, data.info.width))
    data = np.flipud(data)

    # -1 --> 205
    #  0 --> 255
    # 100--> 0
    image = np.where(data == -1, 205, 255 - (255.0 * data / 100)).astype(np.uint8)

    #cv2.imshow("cv2_map", data)
    return image


def cv2_to_map_server(image):
    image = np.flipud(image)

    # 205 --> -1
    # 255 -->  0
    #   0 --> 100
    data = np.where(image == 205, -1, (255 - image) * 100.0 / 255).astype(np.int8).flatten()
    
    return list(data)
    # pub_data = OccupancyGrid()
    # pub_data.header.frame_id = "map"
    # pub_data.header.stamp = rospy.Time.now()
    # pub_data.info = map_data.info
    # pub_data.data = list(data)

    # print(len(pub_data.data), pub_data.info.height * pub_data.info.width)

    # map_publisher.publish(pub_data)
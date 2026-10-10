#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray

class LidarDirectionMonitor(object):
    def __init__(self):
        rospy.init_node('lidar_direction_monitor', anonymous=True)
        
        # 订阅原始激光雷达数据
        self.scan_sub = rospy.Subscriber('/scan_filtered', LaserScan, self.scan_callback)
        
        # 创建方向距离发布器
        self.distance_pub = rospy.Publisher('/lidar_direction_distances', Float32MultiArray, queue_size=10)
        
        # 初始化参数
        self.directions = {
            'front': {'min_angle': 0, 'max_angle': 180},    # 前向：0°-180°
            'back': {'min_angle': 180, 'max_angle': 360},   # 后向：180°-360°
            'left': {'min_angle': 45, 'max_angle': 135},    # 左向：45°-135°
            'right': {'min_angle': 225, 'max_angle': 315}   # 右向：225°-315°
        }
        
        # 存储方向名称列表
        self.direction_names = ['front', 'back', 'left', 'right']
        
        # 初始化方向距离数组
        self.direction_distances = [0.0] * len(self.direction_names)
        
        # 设置日志信息
        rospy.loginfo("激光雷达方向距离监测节点已启动")
    
    def scan_callback(self, scan_msg):
        # 将激光数据转换为numpy数组
        ranges = np.array(scan_msg.ranges)
        
        # 计算每个激光束对应的角度
        angles = np.degrees(scan_msg.angle_min + 
                           np.arange(len(ranges)) * scan_msg.angle_increment)
        
        # 处理每个方向
        for idx, direction in enumerate(self.direction_names):
            dir_config = self.directions[direction]
            
            # 创建角度掩码（处理角度环绕问题）
            angle_mask = ((angles >= dir_config['min_angle']) & 
                         (angles <= dir_config['max_angle'])) | \
                        ((dir_config['max_angle'] < dir_config['min_angle']) & 
                         ((angles >= dir_config['min_angle']) | 
                          (angles <= dir_config['max_angle'])))
            
            # 提取有效范围数据
            dir_ranges = ranges[angle_mask]
            
            # 过滤无效值
            valid_ranges = dir_ranges[np.isfinite(dir_ranges)]
            valid_ranges = valid_ranges[valid_ranges > scan_msg.range_min]
            valid_ranges = valid_ranges[valid_ranges < scan_msg.range_max]
            
            # 计算最小距离
            min_distance = np.min(valid_ranges) if valid_ranges.size > 0 else float('inf')
            self.direction_distances[idx] = min_distance
        
        # 发布方向距离数据
        self.publish_distances()
    
    def publish_distances(self):
        # 创建数据消息
        distance_msg = Float32MultiArray()
        distance_msg.data = self.direction_distances
        
        # 打印实时距离信息
        log_str = "方向距离: 前=%.2f m 后=%.2f m 左=%.2f m 右=%.2f m" % tuple(self.direction_distances)
        rospy.loginfo(log_str)
        
        # 发布数据
        self.distance_pub.publish(distance_msg)

if __name__ == '__main__':
    try:
        LidarDirectionMonitor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
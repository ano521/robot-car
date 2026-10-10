#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

def main():
    # 初始化ROS节点
    rospy.init_node('pub_tts_node', anonymous=True)
    
    # 创建发布者
    pub = rospy.Publisher('text_to_speech', String, queue_size=10)
    
    # 设置发布频率
    rate = rospy.Rate(10)  # 10 Hz
    
    while not rospy.is_shutdown():
        # 获取用户输入
        text = input("Enter text to publish: ")
        
        # 发布消息
        pub.publish(text)
        
        # 打印发布的消息
        rospy.loginfo("Published: %s", text)
        
        # 控制发布频率
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
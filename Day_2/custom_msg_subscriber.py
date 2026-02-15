#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import random
from hello_vscode.msg import MyCustomMsg  # 替换为你的包名

def custom_msg_publisher():
    # 初始化节点
    rospy.init_node('custom_msg_publisher', anonymous=True)
    
    # 创建发布者，发布到"custom_topic"话题，消息类型为MyCustomMsg
    pub = rospy.Publisher('custom_topic', MyCustomMsg, queue_size=10)
    
    rate = rospy.Rate(1)  # 1Hz发布频率
    seq = 0
    
    rospy.loginfo("自定义消息发布者已启动，开始发布消息...")
    
    while not rospy.is_shutdown():
        # 创建消息对象并填充字段
        msg = MyCustomMsg()
        msg.seq = seq
        msg.description = "传感器数据#{}".format(seq)  # 使用format替代f-string
        msg.value = random.uniform(0.0, 100.0)  # 生成0-100的随机数
        msg.is_valid = seq % 2 == 0  # 偶数序列号为有效，奇数为无效
        
        # 发布消息
        pub.publish(msg)
        
        # 打印发布信息（使用format替代f-string）
        rospy.loginfo("发布消息 - 序列号: {0}, 数值: {1:.2f}".format(
            msg.seq, msg.value
        ))
        
        seq += 1
        rate.sleep()

if __name__ == '__main__':
    try:
        custom_msg_publisher()
    except rospy.ROSInterruptException:
        pass
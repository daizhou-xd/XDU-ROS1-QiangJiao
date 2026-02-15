#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from hello_vscode.msg import MyCustomMsg  # 替换为你的包名

def callback(msg):
    """消息回调函数，处理接收到的自定义消息"""
    # 使用format()方法替代f-string，兼容低版本Python
    rospy.loginfo("\n接收到自定义消息:"
                  "\n  序列号: {0}"
                  "\n  描述: {1}"
                  "\n  数值: {2:.2f}"
                  "\n  有效状态: {3}".format(
                      msg.seq,
                      msg.description,
                      msg.value,
                      "有效" if msg.is_valid else "无效"
                  ))

def custom_msg_subscriber():
    # 初始化节点
    rospy.init_node('custom_msg_subscriber', anonymous=True)
    
    # 创建订阅者，订阅"custom_topic"话题，消息类型为MyCustomMsg
    rospy.Subscriber('custom_topic', MyCustomMsg, callback)
    
    rospy.loginfo("自定义消息订阅者已启动，等待接收消息...")
    
    # 保持节点运行，直到被关闭
    rospy.spin()

if __name__ == '__main__':
    try:
        custom_msg_subscriber()
    except rospy.ROSInterruptException:
        pass
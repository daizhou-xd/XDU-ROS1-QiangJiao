#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from hello_vscode.srv import ComputeService, ComputeServiceResponse  # 替换为你的包名

def handle_compute_request(req):
    """处理计算请求的回调函数"""
    # 计算 (a - b) × (c + d)
    result = (req.a - req.b) * (req.c + req.d)
    
    # 使用format替代f-string
    rospy.loginfo("收到请求: a={0}, b={1}, c={2}, d={3}".format(
        req.a, req.b, req.c, req.d
    ))
    rospy.loginfo("计算结果: {0}".format(result))
    
    return ComputeServiceResponse(result)

def compute_server():
    # 初始化节点
    rospy.init_node('compute_server')
    
    # 创建服务，注册回调函数
    rospy.Service('compute_service', ComputeService, handle_compute_request)
    
    rospy.loginfo("计算服务已启动，等待请求...")
    rospy.spin()  # 保持服务运行

if __name__ == "__main__":
    try:
        compute_server()
    except rospy.ROSInterruptException:
        pass
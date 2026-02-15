#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from hello_vscode.srv import ComputeService  # 替换为你的包名

def compute_client(a, b, c, d):
    """客户端函数，发送计算请求并返回结果"""
    # 等待服务可用
    rospy.wait_for_service('compute_service')
    
    try:
        # 创建服务客户端
        compute = rospy.ServiceProxy('compute_service', ComputeService)
        
        # 发送请求并获取响应
        response = compute(a, b, c, d)
        return response.result
    
    except rospy.ServiceException as e:
        # 使用format替代f-string
        rospy.logerr("服务调用失败: {0}".format(e))
        return None

if __name__ == "__main__":
    try:
        # 初始化节点
        rospy.init_node('compute_client')
        
        # 可以修改这里的四个数字进行测试
        a = 20.0
        b = 6.0
        c = 6.0
        d = 6.0
        
        # 使用format替代f-string
        rospy.loginfo("发送请求: a={0}, b={1}, c={2}, d={3}".format(a, b, c, d))
        result = compute_client(a, b, c, d)
        
        if result is not None:
            # 使用format替代f-string
            rospy.loginfo("收到响应: (a - b) × (c + d) = {0}".format(result))
    
    except rospy.ROSInterruptException:
        pass
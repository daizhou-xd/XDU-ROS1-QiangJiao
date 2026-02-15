#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from turtlesim.srv import Spawn
from std_srvs.srv import Empty

def spawn_turtles():
    # 初始化节点
    rospy.init_node('spawn_multiple_turtles', anonymous=True)
    
    rospy.loginfo("等待turtlesim服务启动...")
    
    # 等待必要的服务可用
    rospy.wait_for_service('/spawn')
    rospy.wait_for_service('/clear')
    
    try:
        # 创建服务客户端
        spawn = rospy.ServiceProxy('/spawn', Spawn)
        clear = rospy.ServiceProxy('/clear', Empty)
        
        # 1. 通过参数服务器修改背景色（兼容所有ROS版本）
        # 参数说明：r, g, b范围为0-1.0（需除以255转换）
        rospy.set_param('/turtlesim/background_r', 135.0)  # 红色分量
        rospy.set_param('/turtlesim/background_g', 206.0)  # 绿色分量
        rospy.set_param('/turtlesim/background_b', 235.0)  # 蓝色分量
        
        # 重启turtlesim使背景色生效（通过重置服务）
        rospy.wait_for_service('/reset')
        reset = rospy.ServiceProxy('/reset', Empty)
        reset()
        rospy.loginfo("背景色已修改为浅蓝色")
        
        # 2. 清除轨迹
        clear()
        
        # 3. 定义4只乌龟的参数
        turtles = [
            {"name": "turtle_1", "x": 2.0, "y": 2.0, "theta": 0.0},
            {"name": "turtle_2", "x": 8.0, "y": 2.0, "theta": 1.57},
            {"name": "turtle_3", "x": 8.0, "y": 8.0, "theta": 3.14},
            {"name": "turtle_4", "x": 2.0, "y": 8.0, "theta": 4.71}
        ]
        
        # 4. 生成乌龟
        for turtle in turtles:
            rospy.loginfo("生成乌龟: {0}".format(turtle["name"]))
            spawn(
                x=turtle["x"],
                y=turtle["y"],
                theta=turtle["theta"],
                name=turtle["name"]
            )
            rospy.sleep(0.5)
        
        rospy.loginfo("所有乌龟生成完成！")
    
    except rospy.ServiceException as e:
        rospy.logerr("服务调用失败: {0}".format(e))

if __name__ == "__main__":
    try:
        spawn_turtles()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
import time

def draw_square():
    # 初始化节点
    rospy.init_node("turtle_draw_square", anonymous=True)
    # 创建发布者，发布速度指令
    pub = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=1000)
    rate = rospy.Rate(10)  # 10Hz控制频率
    msg = Twist()
    
    # 正方形参数设置
    linear_speed = 0.8     # 直线运动速度
    angular_speed = 1.0    # 旋转速度（弧度/秒）
    side_length = 2.0      # 正方形边长
    forward_time = side_length / linear_speed  # 移动一条边的时间
    turn_time = (3.14159 / 2) / angular_speed  # 旋转90度的时间（π/2弧度）
    
    rospy.loginfo("开始绘制正方形...")
    
    # 循环4次，绘制4条边
    for _ in range(4):
        # 1. 向前移动（绘制一条边）
        msg.linear.x = linear_speed
        msg.angular.z = 0.0
        start_time = time.time()
        while not rospy.is_shutdown() and (time.time() - start_time) < forward_time:
            pub.publish(msg)
            rate.sleep()
        
        # 2. 停止前进
        msg.linear.x = 0.0
        pub.publish(msg)
        rospy.sleep(0.5)  # 短暂停顿
        
        # 3. 旋转90度
        msg.angular.z = angular_speed
        start_time = time.time()
        while not rospy.is_shutdown() and (time.time() - start_time) < turn_time:
            pub.publish(msg)
            rate.sleep()
        
        # 4. 停止旋转
        msg.angular.z = 0.0
        pub.publish(msg)
        rospy.sleep(0.5)  # 短暂停顿
    
    # 最终停止所有运动
    msg.linear.x = 0.0
    msg.angular.z = 0.0
    pub.publish(msg)
    rospy.loginfo("正方形绘制完成！")

if __name__ == "__main__":
    try:
        draw_square()  # 直接调用画正方形函数
    except rospy.ROSInterruptException:
        pass
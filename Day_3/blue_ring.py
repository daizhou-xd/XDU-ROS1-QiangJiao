#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import time
from geometry_msgs.msg import Twist
from turtlesim.srv import Spawn, SetPen
from std_srvs.srv import Empty

def main():
    # 初始化节点
    rospy.init_node('blue_ring_controller')
    
    # 等待服务可用
    rospy.wait_for_service('/spawn')
    rospy.wait_for_service('/clear')
    
    # 创建清除轨迹服务
    clear = rospy.ServiceProxy('/clear', Empty)
    clear()
    
    try:
        # 生成蓝色乌龟（上排左）
        spawn = rospy.ServiceProxy('/spawn', Spawn)
        spawn(3.0, 6.0, 0, 'blue_turtle')  # x, y, theta, name
        
        # 设置画笔颜色（蓝色: RGB(0,0,255)）
        rospy.wait_for_service('/blue_turtle/set_pen')
        set_pen = rospy.ServiceProxy('/blue_turtle/set_pen', SetPen)
        set_pen(0, 0, 255, 2, 0)  # r, g, b, width, off
        
        # 创建速度发布者
        pub = rospy.Publisher('/blue_turtle/cmd_vel', Twist, queue_size=10)
        rate = rospy.Rate(10)
        vel = Twist()
        
        # 圆周运动参数（线速度1.0，角速度1.0）
        vel.linear.x = 1.5
        vel.angular.z = 1.0
        
        # 持续发布速度指令
        while not rospy.is_shutdown():
            pub.publish(vel)
            rate.sleep()
            
    except rospy.ServiceException as e:
        # 使用format()替代f-string
        rospy.loginfo("服务调用失败: {}".format(str(e)))
    except Exception as e:
        rospy.loginfo("发生错误: {}".format(str(e)))

if __name__ == '__main__':
    main()
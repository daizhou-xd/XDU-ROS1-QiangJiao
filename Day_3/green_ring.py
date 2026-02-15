#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import time
from geometry_msgs.msg import Twist
from turtlesim.srv import Spawn, SetPen
from std_srvs.srv import Empty

def main():
    rospy.init_node('green_ring_controller')
    
    rospy.wait_for_service('/spawn')
    rospy.wait_for_service('/clear')
    
    clear = rospy.ServiceProxy('/clear', Empty)
    clear()
    
    try:
        # 生成绿色乌龟（下排右）
        spawn = rospy.ServiceProxy('/spawn', Spawn)
        spawn(6.75, 4.5, 0, 'green_turtle')
        
        # 设置画笔颜色（绿色: RGB(0,255,0)）
        rospy.wait_for_service('/green_turtle/set_pen')
        set_pen = rospy.ServiceProxy('/green_turtle/set_pen', SetPen)
        set_pen(0, 255, 0, 2, 0)
        
        pub = rospy.Publisher('/green_turtle/cmd_vel', Twist, queue_size=10)
        rate = rospy.Rate(10)
        vel = Twist()
        
        vel.linear.x = 1.5
        vel.angular.z = 1.0
        
        while not rospy.is_shutdown():
            pub.publish(vel)
            rate.sleep()
            
    except rospy.ServiceException as e:
        rospy.loginfo("服务调用失败: {}".format(str(e)))
    except Exception as e:
        rospy.loginfo("发生错误: {}".format(str(e)))

if __name__ == '__main__':
    main()
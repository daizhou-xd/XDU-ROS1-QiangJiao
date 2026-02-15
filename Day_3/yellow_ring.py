#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import time
from geometry_msgs.msg import Twist
from turtlesim.srv import Spawn, SetPen
from std_srvs.srv import Empty

def main():
    rospy.init_node('yellow_ring_controller')
    
    rospy.wait_for_service('/spawn')
    rospy.wait_for_service('/clear')
    
    clear = rospy.ServiceProxy('/clear', Empty)
    clear()
    
    try:
        # 生成黄色乌龟（下排左）
        spawn = rospy.ServiceProxy('/spawn', Spawn)
        spawn(4.25, 4.5, 0, 'yellow_turtle')
        
        # 设置画笔颜色（黄色: RGB(255,255,0)）
        rospy.wait_for_service('/yellow_turtle/set_pen')
        set_pen = rospy.ServiceProxy('/yellow_turtle/set_pen', SetPen)
        set_pen(255, 255, 0, 2, 0)
        
        pub = rospy.Publisher('/yellow_turtle/cmd_vel', Twist, queue_size=10)
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
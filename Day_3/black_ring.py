#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import time
from geometry_msgs.msg import Twist
from turtlesim.srv import Spawn, SetPen
from std_srvs.srv import Empty

def main():
    rospy.init_node('black_ring_controller')
    
    rospy.wait_for_service('/spawn')
    rospy.wait_for_service('/clear')
    
    clear = rospy.ServiceProxy('/clear', Empty)
    clear()
    
    try:
        # 生成黑色乌龟（上排中）
        spawn = rospy.ServiceProxy('/spawn', Spawn)
        spawn(5.5, 6.0, 0, 'black_turtle')
        
        # 设置画笔颜色（黑色: RGB(0,0,0)）
        rospy.wait_for_service('/black_turtle/set_pen')
        set_pen = rospy.ServiceProxy('/black_turtle/set_pen', SetPen)
        set_pen(0, 0, 0, 2, 0)
        
        pub = rospy.Publisher('/black_turtle/cmd_vel', Twist, queue_size=10)
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
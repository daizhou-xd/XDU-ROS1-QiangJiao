#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

def circle_move():
    rospy.init_node('turtle_circle',anonymous=True)
    pub=rospy.Publisher('/turtle1/cmd_vel',Twist,queue_size=10)
    rate=rospy.Rate(10)
    vel_msg=Twist()
    vel_msg.linear.x=1.0
    vel_msg.angular.z=1.0
    while not rospy.is_shutdown():
        pub.publish(vel_msg)
        rate.sleep()
if __name__=='__main__':
    try:
        circle_move()
    except rospy.ROSInitException:
        pass

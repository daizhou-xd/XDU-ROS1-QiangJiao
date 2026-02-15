#!/usr/bin/env python
import rospy
from turtlesim.msg import Pose

def callback(msg):
    print("x: {:.2f}, y: {:.2f}, theta: {:.2f}".format(msg.x, msg.y, msg.theta))

def main():
    rospy.init_node('turtle_pose_listener')
    rospy.Subscriber('/turtle1/pose', Pose, callback)
    rospy.spin()

if __name__ == '__main__':
    main()

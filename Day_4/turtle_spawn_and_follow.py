#!/usr/bin/env python
# -*- coding: utf-8 -*-
import roslib
roslib.load_manifest('learning_tf')  # 替换为你的包名
import rospy
import math
import tf
import geometry_msgs.msg
import turtlesim.srv

def spawn_turtle(x, y, theta, name):
    """调用spawn服务生成新海龟"""
    rospy.wait_for_service('spawn')
    try:
        spawner = rospy.ServiceProxy('spawn', turtlesim.srv.Spawn)
        spawner(x, y, theta, name)
        rospy.loginfo("生成海龟: %s" % name)
    except rospy.ServiceException as e:
        rospy.logerr("生成海龟失败: %s" % e)

def follow_turtle(follower_name, min_distance):
    """跟随者逻辑（单独函数，用于线程调用）"""
    # 初始化速度发布器
    vel_pub = rospy.Publisher(
        '/%s/cmd_vel' % follower_name,
        geometry_msgs.msg.Twist,
        queue_size=10
    )
    # TF监听器
    listener = tf.TransformListener()
    rate = rospy.Rate(10.0)  # 10Hz循环

    # 等待TF变换可用（官方例程标准做法）
    listener.waitForTransform(
        '/%s' % follower_name,
        '/turtle1',
        rospy.Time(0),
        rospy.Duration(1.0)
    )

    while not rospy.is_shutdown():
        try:
            # 获取turtle1相对于跟随者的最新变换（官方例程核心逻辑）
            now = rospy.Time.now()
            # 等待1秒内的变换（解决时间同步问题）
            listener.waitForTransform(
                '/%s' % follower_name,
                '/turtle1',
                now,
                rospy.Duration(1.0)
            )
            (trans, rot) = listener.lookupTransform(
                '/%s' % follower_name,
                '/turtle1',
                now
            )
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        # 计算距离
        distance = math.sqrt(trans[0] **2 + trans[1]** 2)
        cmd = geometry_msgs.msg.Twist()

        # 官方例程速度控制逻辑（增加最小距离判断）
        if distance > min_distance:
            cmd.linear.x = 0.5 * distance  # 线速度与距离成正比
            cmd.angular.z = 4 * math.atan2(trans[1], trans[0])  # 角速度控制转向
        else:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

        vel_pub.publish(cmd)
        rate.sleep()

if __name__ == '__main__':
    rospy.init_node('turtle_spawn_and_follow')

    # 1. 生成等边三角形分布的3只海龟（以turtle1为中心）
    turtle1_x = 5.544444561  # turtlesim默认初始位置
    turtle1_y = 5.544444561
    radius = 2.0  # 与中心的距离
    angles = [0, 2*math.pi/3, 4*math.pi/3]  # 120°间隔
    follower_names = ['turtle2', 'turtle3', 'turtle4']
    min_distances = [1.0, 1.5, 2.0]  # 不同最小跟踪距离

    for i in range(3):
        x = turtle1_x + radius * math.cos(angles[i])
        y = turtle1_y + radius * math.sin(angles[i])
        spawn_turtle(x, y, 0.0, follower_names[i])

    # 2. 为每个跟随者启动独立线程（参考官方多线程处理方式）
    import threading
    for name, dist in zip(follower_names, min_distances):
        # 用target传递函数，args传递参数，避免daemon参数（兼容所有Python版本）
        thread = threading.Thread(target=follow_turtle, args=(name, dist))
        thread.start()  # 启动线程

    rospy.spin()
#!/usr/bin/env python
# -*- coding: utf-8 -*-
import roslib
roslib.load_manifest('learning_tf')  # 替换为你的包名
import rospy
import tf
from turtlesim.msg import Pose

def handle_turtle_pose(msg, turtlename):
    # 创建TF广播器
    br = tf.TransformBroadcaster()
    # 发送坐标变换：海龟坐标系 -> 世界坐标系
    br.sendTransform(
        (msg.x, msg.y, 0.0),  # 平移分量（x,y,z）
        # 将欧拉角（滚转,俯仰,偏航）转换为四元数
        tf.transformations.quaternion_from_euler(0, 0, msg.theta),
        rospy.Time.now(),    # 时间戳
        turtlename,          # 子坐标系（海龟自身坐标系）
        "world"              # 父坐标系（世界坐标系）
    )

if __name__ == '__main__':
    rospy.init_node('turtle_tf_broadcaster')
    # 从参数服务器获取当前海龟名称（支持多海龟广播）
    turtlename = rospy.get_param('~turtle')
    # 订阅该海龟的位姿话题，回调函数中广播TF
    rospy.Subscriber(
        '/%s/pose' % turtlename,
        Pose,
        handle_turtle_pose,
        turtlename  # 传递额外参数（海龟名称）
    )
    rospy.spin()
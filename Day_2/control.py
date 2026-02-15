#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import sys, select, tty, termios
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

# 保存当前乌龟位姿
current_pose = Pose()

# 键盘按键与速度映射
key_vel_map = {
    'w': (0.5, 0),     # 前进
    's': (-0.5, 0),    # 后退
    'a': (0, 0.5),     # 左转
    'd': (0, -0.5),    # 右转
    'x': (0, 0),       # 停止
    'q': None          # 退出
}

def pose_callback(pose):
    """位姿回调函数，更新当前位姿并打印"""
    global current_pose
    current_pose = pose
    
    # 清除当前行并打印新信息（终端显示优化）
    sys.stdout.write('\r')
    sys.stdout.write("当前位姿 - X: {:.2f}, Y: {:.2f}, 朝向: {:.2f} 弧度".format(
        current_pose.x, current_pose.y, current_pose.theta))
    sys.stdout.flush()

def get_key():
    """获取键盘输入（非阻塞模式）"""
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def manual_control():
    """手动控制主函数"""
    # 初始化节点
    rospy.init_node('turtle_manual_control', anonymous=True)
    
    # 创建速度发布者
    vel_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    
    # 订阅位姿话题
    rospy.Subscriber('/turtle1/pose', Pose, pose_callback)
    
    rate = rospy.Rate(10)  # 10Hz
    twist = Twist()
    
    rospy.loginfo("开始手动控制乌龟...")
    rospy.loginfo("控制键: W(前) S(后) A(左) D(右) X(停) Q(退出)")
    
    while not rospy.is_shutdown():
        key = get_key()
        
        # 处理按键输入
        if key in key_vel_map:
            if key_vel_map[key] is None:  # 退出
                break
            
            # 设置速度
            linear, angular = key_vel_map[key]
            twist.linear.x = linear
            twist.angular.z = angular
        else:
            # 未知按键保持当前速度
            pass
        
        # 发布速度指令
        vel_pub.publish(twist)
        rate.sleep()
    
    # 退出前停止乌龟
    twist.linear.x = 0
    twist.angular.z = 0
    vel_pub.publish(twist)
    rospy.loginfo("\n程序已退出")

if __name__ == "__main__":
    # 保存终端设置用于恢复
    settings = termios.tcgetattr(sys.stdin)
    try:
        manual_control()
    except rospy.ROSInterruptException:
        pass
    finally:
        # 恢复终端设置
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
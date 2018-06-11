#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import quaternion_calculate as qc
from iiwa_msgs.msg import JointPosition
import math


def talker():
    rospy.init_node('calibrate_commander', anonymous=True)
    pub = rospy.Publisher('/iiwa/command/JointPosition', JointPosition, queue_size=100)
    rospy.sleep(1)
    # commend_point=getcommand((-120,0,0,-90,-25,-90,0))  # 标定位置
    # commend_point = getcommand((-140, 30, 25, -80, -60, -80, -20))  # 扎蛋糕标定位置
    # commend_point = get_command_joint((-50, 30, 0, -100, -30, -90, 0))  # 0509师兄实验标定位置
    # commend_point = qc.get_command_joint((110, 0, 0, 70, -10, -90, 0))  # 0511反向位置
    # commend_point = get_command_joint((-50, 30, 0, -100, -30, -90, 0))  #0611彭涛实验标定位置
    commend_point = get_command_joint((0, 0, 0, -90, 0, 90, 0))  # 下垂姿势
    # commend_point = get_command_joint((0, 0, 0, 0, 0, 0, 0))
    pub.publish(commend_point)
    # pub.publish(1)
    rospy.loginfo(commend_point)


def get_command_joint(calibrate_point):
    command_point=JointPosition()
    command_point.position.a1 = calibrate_point[0]/180.0*math.pi
    command_point.position.a2 = calibrate_point[1]/180.0*math.pi
    command_point.position.a3 = calibrate_point[2]/180.0*math.pi
    command_point.position.a4 = calibrate_point[3]/180.0*math.pi
    command_point.position.a5 = calibrate_point[4]/180.0*math.pi
    command_point.position.a6 = calibrate_point[5]/180.0*math.pi
    command_point.position.a7 = calibrate_point[6]/180.0*math.pi
    return command_point

if __name__ == '__main__':
    talker()
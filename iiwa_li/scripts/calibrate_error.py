#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from iiwa_msgs.msg import JointPosition
import quaternion_calculate as qc
from geometry_msgs.msg import PoseStamped
from numpy.linalg import inv
import math

#发送某视觉空间位置点,自动解算TJO,用error.m计算误差
def follow():
    rospy.init_node('calibrate_error', anonymous=True)
    jpub = rospy.Publisher('/iiwa/command/JointPosition', JointPosition, queue_size=100)
    ppub = rospy.Publisher('/iiwa/command/CartesianPose', PoseStamped, queue_size=10)
    rospy.sleep(1)
    TJM = np.loadtxt('/home/lizq/win7share/TJM.txt', delimiter=",")  # mm
    TBO = np.loadtxt('/home/lizq/win7share/TBO.txt', delimiter=",")  # mm
    p1 = np.genfromtxt('/home/lizq/win7share/NDI.txt', delimiter=",")[0]
    print "出发点坐标",p1
    TMB = qc.quat2matrix(p1)
    # backpoint = qc.get_command_joint((0, 0, 0, -90, 0, -90, 0))
    backpoint = qc.get_command_joint((110, 0, 0, 90, 0, -90, 0))
    # backpoint = qc.get_command_joint((0, 0, 0, 0, 0, 0, 0))
    jpub.publish(backpoint)
    # rospy.loginfo(backpoint)
    rospy.sleep(3)
    TJO=TJM.dot(TMB).dot(TBO)
    TJO[0:3][:, 3] /= 1000  # mm->m
    quat = list(qc.matrix2quat(TJO))
    command_point = qc.get_command_pose(quat)
    # rospy.loginfo(command_point)
    ppub.publish(command_point)
    rospy.sleep(3)
    p2 = np.genfromtxt('/home/lizq/win7share/NDI.txt', delimiter=",")[0]
    print "回归点坐标",p2
    d1 = np.mat(p1[0:3])
    d2 = np.mat(p2[0:3])
    print "位置偏差", np.sqrt((d1 - d2) * (d1 - d2).T) , "mm"
    try:
        q = math.acos(qc.matrix2quat(qc.quat2matrix(p2).dot(inv(qc.quat2matrix(p1))))[6]) * 2.0 * 180.0 / np.pi
    except RuntimeWarning:
        q = math.acos(qc.matrix2quat(qc.quat2matrix(p1).dot(inv(qc.quat2matrix(p2))))[6]) * 2.0 * 180.0 / np.pi
    print "角度偏差", q ,"°"


if __name__ == '__main__':
    follow()
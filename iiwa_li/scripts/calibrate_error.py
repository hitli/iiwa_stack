#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import quaternion_calculate as qc
from geometry_msgs.msg import PoseStamped


#发送某视觉空间位置点,自动解算TJO,用error.m计算误差
def follow():
    rospy.init_node('follow', anonymous=True)
    rospy.sleep(1)
    pub = rospy.Publisher('/iiwa/command/CartesianPose', PoseStamped, queue_size=10)
    TJM = np.loadtxt('/home/lizq/win7share/TJM.txt', delimiter=",")  # mm
    TBO = np.loadtxt('/home/lizq/win7share/TBO.txt', delimiter=",")  # mm
    TMB = qc.quat2matrix((-52.2300,64.4000,-1181.0500,-0.4117,-0.1769,-0.8114,0.3749))
    #-52.3600, 64.4300, -1181.1899, -0.4115, -0.1773, -0.8114, 0.3749
    TJO=TJM.dot(TMB).dot(TBO)
    #mm->m
    TJO[0][3] /= 1000
    TJO[1][3] /= 1000
    TJO[2][3] /= 1000
    quat = list(qc.matrix2quat(TJO))
    command_point = qc.get_command_pose(quat)
    rospy.loginfo(command_point)
    pub.publish(command_point)


if __name__ == '__main__':
    follow()
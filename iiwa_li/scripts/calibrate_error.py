#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import quaternion_calculate as qc
from numpy.linalg import inv
from geometry_msgs.msg import PoseStamped


def follow():
    rospy.init_node('follow', anonymous=True)
    rospy.sleep(1)
    rate = rospy.Rate(10)
    pub = rospy.Publisher('/iiwa/command/CartesianPose', PoseStamped, queue_size=10)
    TJM = np.loadtxt('/home/lizq/win7share/TJM.txt', delimiter=",")  # mm
    TBO = np.loadtxt('/home/lizq/win7share/TBO.txt', delimiter=",")  # mm
    while not rospy.is_shutdown():
        TMB = qc.quat2matrix((-52.2300,64.4000,-1181.0500,-0.4117,-0.1769,-0.8114,0.3749))
        #-52.3600, 64.4300, -1181.1899, -0.4115, -0.1773, -0.8114, 0.3749
        # TMB = qc.quat2matrix(np.loadtxt('/home/lizq/win7share/NDI.txt', delimiter=","))
        TJO=TJM.dot(TMB).dot(TBO)
        #mm->m
        TJO[0][3] /= 1000
        TJO[1][3] /= 1000
        TJO[2][3] /= 1000
        quat = list(qc.matrix2quat(TJO))
        command_point = qc.get_command_pose(quat)
        rospy.loginfo(command_point)
        pub.publish(command_point)
        rate.sleep()


if __name__ == '__main__':
    follow()
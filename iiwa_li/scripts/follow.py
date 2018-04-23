#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
import numpy as np
from iiwa_msgs.msg import JointPosition
import quaternion_calculate as qc
from numpy.linalg import inv
from geometry_msgs.msg import PoseStamped


def follow():
    # rospy.init_node('follow', anonymous=True)
    # pub = rospy.Publisher('/iiwa/command/CartesianPose', PoseStamped, queue_size=10)
    # rate = rospy.Rate(10)
    TJM = np.loadtxt('/home/lizq/win7share/TJM.txt', delimiter=",")  # mm
    # TBO = np.loadtxt('/home/lizq/win7share/TBO.txt', delimiter=",")  # mm
    TNB = np.loadtxt('/home/lizq/win7share/TNB.txt', delimiter=",")  # mm
    # TNO = TNB.dot(TBO)
    TON = np.loadtxt('/home/lizq/win7share/TON.txt', delimiter=",")  # mm
    TCP = np.loadtxt('/home/lizq/win7share/TCP.txt', delimiter=",").tolist()  # m
    TNO = inv(TON)
    TNN = np.array([[1.0,0.0,0.0,-13.5318],[0.0,1.0,0.0,0.50804],[0.0,0.0,1.0,-153.66507],[0.0,0.0,0.0,1.0]])
    while not rospy.is_shutdown():
        try:
            TMN = qc.quat2matrix(np.loadtxt('/home/lizq/win7share/NDI.txt', delimiter=",", skiprows=1).tolist())
            # TMN = qc.quat2matrix(np.loadtxt('/home/lizq/win7share/NDI.txt', delimiter=",").tolist())
            TMN = TMN.dot(TNN)#更正钢针位置
            TMN = TMN.dot(TNB)
            TJO=TJM.dot(TMN).dot(TNO)
            #mm->m
            TJO[0][3] /= 1000
            TJO[1][3] /= 1000
            TJO[2][3] /= 1000
            quat =list(qc.matrix2quat(TJO))
            quat[3:] = TCP[3:]
            print qc.matrix2quat(TMN)
            # command_point = qc.get_command_pose(quat)
            # rospy.loginfo(command_point)
            # pub.publish(command_point)
            rate.sleep()
        except :
            pass
        finally:
            # print 'waiting'
            pass





if __name__ == '__main__':
    follow()
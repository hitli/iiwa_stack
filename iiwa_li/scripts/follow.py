#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import quaternion_calculate as qc
from numpy.linalg import inv
from geometry_msgs.msg import PoseStamped


def follow():
    rospy.init_node('follow', anonymous=True)
    pub = rospy.Publisher('/iiwa/command/CartesianPose', PoseStamped, queue_size=10)
    rate = rospy.Rate(3)
    TJM = np.loadtxt('/home/lizq/win7share/TJM.txt', delimiter=",")  # mm
    TON = np.loadtxt('/home/lizq/win7share/TON.txt', delimiter=",")  # mm
    TNO = inv(TON)
    TCP = np.loadtxt('/home/lizq/win7share/TCP.txt', delimiter=",").tolist()  # m
    TNN = np.array([[1.0,0.0,0.0,-13.5318],
                    [0.0,1.0,0.0,0.50804],
                    [0.0,0.0,1.0,-153.66507],
                    [0.0,0.0,0.0,1.0]])
    while not rospy.is_shutdown():
        try:
            TMN = qc.quat2matrix(np.loadtxt('/home/lizq/win7share/NDI.txt', delimiter=",", skiprows=1).tolist())
            TMN = TMN.dot(TNN)#更正钢针位姿
            TJN = TJM.dot(TMN)
            quat = list(qc.matrix2quat(TJN))#使得TCP姿态不变，被动刚体朝向NDI，只做位移
            quat[3:] = TCP[3:]
            TJN = qc.quat2matrix(quat)
            TJO=TJN.dot(TNO)
            #mm->m
            TJO[0:3][:, 3] /= 1000
            command_point = qc.get_command_pose(qc.matrix2quat(TJO))
            rospy.loginfo(command_point)
            pub.publish(command_point)
            rate.sleep()
        except BaseException,e:
            # print e
            pass
        finally:
            # print 'waiting'
            pass





if __name__ == '__main__':
    follow()
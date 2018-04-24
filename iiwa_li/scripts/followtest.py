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
    # TNB = np.loadtxt('/home/lizq/win7share/TNB.txt', delimiter=",")  # mm
    # TNO = TNB.dot(TBO)
    TBO = np.loadtxt('/home/lizq/win7share/TBO.txt', delimiter=",")
    TON = np.loadtxt('/home/lizq/win7share/TON.txt', delimiter=",")  # mm
    TNO = inv(TON)
    TCP = np.loadtxt('/home/lizq/win7share/TCP.txt', delimiter=",").tolist()  # m
    TNN = np.array([[1.0,0.0,0.0,-13.5318],[0.0,1.0,0.0,0.50804],[0.0,0.0,1.0,-153.66507],[0.0,0.0,0.0,1.0]])
    while not rospy.is_shutdown():
        try:
            ndi = np.loadtxt('/home/lizq/win7share/NDI.txt', delimiter=",")
            # TMN = qc.quat2matrix(np.loadtxt('/home/lizq/win7share/NDI.txt', delimiter=",", skiprows=1).tolist())
            TMB = qc.quat2matrix(ndi[1].tolist)
            print 'TMB',ndi[1]
            # TMB = qc.quat2matrix(np.loadtxt('/home/lizq/win7share/NDI.txt', delimiter=",").tolist())
            print 'TMN',ndi[2]
            TMN = qc.quat2matrix(ndi[2].tolist)
            TMN = TMN.dot(TNN)#更正钢针位置
            print 'TMN',qc.matrix2quat(TMN)
            print '钢针TMO',qc.matrix2quat(TMN.dot(TNO))
            print '刚体TMO',qc.matrix2quat(TMB.dot(TBO))
            TJO=TJM.dot(TMN).dot(TNO)
            #mm->m
            TJO[0][3] /= 1000
            TJO[1][3] /= 1000
            TJO[2][3] /= 1000
            quat =list(qc.matrix2quat(TJO))
            quat[3:] = TCP[3:]
            command_point = qc.get_command_pose(quat)
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
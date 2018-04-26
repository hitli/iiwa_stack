#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import quaternion_calculate as qc
from numpy.linalg import inv
from geometry_msgs.msg import PoseStamped


############################覆盖被动刚体，采集一行钢针NDI数据跟随####################################
def follow():
    rospy.init_node('follow', anonymous=True)
    pub = rospy.Publisher('/iiwa/command/CartesianPose', PoseStamped, queue_size=100)
    rate = rospy.Rate(10)  # smartservo 20ms
    TJM = np.loadtxt('/home/lizq/win7share/TJM.txt', delimiter=",")  # mm
    TON = np.loadtxt('/home/lizq/win7share/TON.txt', delimiter=",")  # mm
    TNO = inv(TON)
    TCP = np.loadtxt('/home/lizq/win7share/TCP.txt', delimiter=",").tolist()  # m
    TNN = np.array([[1.0,0.0,0.0,-13.5318],[0.0,1.0,0.0,0.50804],[0.0,0.0,1.0,-153.66507],[0.0,0.0,0.0,1.0]])
    while not rospy.is_shutdown():
        try:
            TMN = qc.quat2matrix(np.loadtxt('/home/lizq/win7share/NDI.txt', delimiter=",").tolist())
            TMN = TMN.dot(TNN)#更正钢针位姿
            TJN = TJM.dot(TMN)
            quat = list(qc.matrix2quat(TJN))
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
            print e
            pass
        finally:
            print 'waiting'
            pass





if __name__ == '__main__':
    follow()
#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import quaternion_calculate as qc
from numpy.linalg import inv
from geometry_msgs.msg import PoseStamped
from math import isnan


############################NDI.txt第二行340####################################
def follow():
    rospy.init_node('follow', anonymous=True)
    pub = rospy.Publisher('/iiwa/command/CartesianPose', PoseStamped, queue_size=100)
    rate = rospy.Rate(50)  # smartservo 可以达到 20ms 50hz
    tjm = np.loadtxt('/home/lizq/win7share/TJM.txt', delimiter=",")  # mm
    ton = np.loadtxt('/home/lizq/win7share/TON.txt', delimiter=",")  # mm
    tno = inv(ton)
    tcp = np.loadtxt('/home/lizq/win7share/TCP.txt', delimiter=",").tolist()  # m
    tgg = np.loadtxt('/home/lizq/win7share/TGG.txt', delimiter=",")
    while not rospy.is_shutdown():
        try:
            ndi = np.genfromtxt('/home/lizq/win7share/NDI.txt', delimiter=",")
            if isnan(ndi[1][0]):
                print "waiting"
            else:
                tmg = qc.quat2matrix(ndi[1].tolist())
                tmg = tmg.dot(tgg)  # 更正钢针位姿
                tjn = tjm.dot(tmg)  # 将钢针针尖位置变换至基座坐标系下
                quat = list(qc.matrix2quat(tjn))
                quat[3:] = tcp[3:]  # 使得TCP姿态不变，被动刚体朝向NDI，只做位移
                tjn = qc.quat2matrix(quat)
                tjo = tjn.dot(tno)
                tjo[0:3][:, 3] /= 1000  # mm->m
                command_point = qc.get_command_pose(qc.matrix2quat(tjo))
                rospy.loginfo(command_point)
                pub.publish(command_point)
                rate.sleep()
        except:
            pass


if __name__ == '__main__':
    follow()
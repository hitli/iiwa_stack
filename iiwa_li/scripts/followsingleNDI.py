#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import quaternion_calculate as qc
from numpy.linalg import inv
from geometry_msgs.msg import PoseStamped


############################覆盖被动刚体449，采集一行钢针NDI数据跟随####################################
def follow():
    rospy.init_node('follow', anonymous=True)
    pub = rospy.Publisher('/iiwa/command/CartesianPose', PoseStamped, queue_size=100)
    rate = rospy.Rate(10)  # smartservo 20ms
    tjm = np.loadtxt('/home/lizq/win7share/TJM.txt', delimiter=",")  # mm
    ton = np.loadtxt('/home/lizq/win7share/TON.txt', delimiter=",")  # mm
    tno = inv(ton)
    tcp = np.loadtxt('/home/lizq/win7share/TCP.txt', delimiter=",").tolist()  # m
    tgg = np.loadtxt('/home/lizq/win7share/TGG.txt', delimiter=",")
    while not rospy.is_shutdown():
        try:
            tmg = qc.quat2matrix(np.loadtxt('/home/lizq/win7share/NDI.txt', delimiter=",").tolist())
            tmg = tmg.dot(tgg)  # 更正钢针位姿
            tjn = tjm.dot(tmg)  # 将钢针针尖位置变换至基座坐标系下
            quat = list(qc.matrix2quat(tjn))
            quat[3:] = tcp[3:]  # 使得TCP姿态不变，被动刚体朝向NDI，只做位移
            tjn = qc.quat2matrix(quat)
            tjo=tjn.dot(tno)
            #mm->m
            tjo[0:3][:, 3] /= 1000
            command_point = qc.get_command_pose(qc.matrix2quat(tjo))
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
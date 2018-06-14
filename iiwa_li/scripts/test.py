#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import time
TCP_position = (0,1)
import rospy
import numpy as np
import quaternion_calculate as qc




def read():
    # with open('/home/lizq/win7share/NDI.txt','r') as f:
    #     print (f.read())
    f=open('/home/lizq/win7share/NDI.txt','r',10)
    flag=1
    print (f.read())
    time.sleep(1)
    print (f.read())

def quat2angle(quat):
    q1=2*(quat[3]*quat[4]+ quat[6]*quat[5])
    q2=quat[6]**2 + quat[3]**2 - quat[4]**2 - quat[5]**2
    q3=-2*(quat[3]*quat[5]- quat[6]*quat[4])
    q4=2*(quat[4]*quat[5]+ quat[6]*quat[3])
    q5=quat[6]**2 - quat[3]**2 - quat[4]**2 + quat[5]**2
    return (quat[0],quat[1],quat[2],math.atan2(q1,q2),math.asin(q3),math.atan2(q4,q5))

def l():
    TNN = np.array([[1.0, 0.0, 0.0, -13.5318],
                    [0.0, 1.0, 0.0, 0.50804],
                    [0.0, 0.0, 1.0, -153.66507],
                    [0.0, 0.0, 0.0, 1.0]])
    TNN[0:3][:,3] /= 1000
    print TNN

def cali():
    print '1'
    time.sleep(10)
    print '2'
    # tf = eng.isprime(37)
    print tf


if __name__ == '__main__':
    # x = (-32.5000,-20.9000,-1128.1801,-0.6784,0.1082,-0.7137,0.1362)
    # y = (-23.3100,-6.0000,-1151.3300,0.8913,0.1172,0.4375,0.0165)
    # print qc.matrix2quat(qc.quat2matrix(x).dot(qc.quat2matrix(y)))
    # print qc.quat_pose_multipy(x,y)
    # print 0.18456656039890573*0.18456656039890573+-0.032630922568744995*-0.032630922568744995+0.19420782694834174*0.19420782694834174+0.9628882217717862*0.9628882217717862
    # print 0.18452686999999995*0.18452686999999995+0.029733259999999984*0.029733259999999984+0.19421673*0.19421673+0.96279567*0.96279567
    TJM = np.loadtxt('/home/lizq/win7share/TJM.txt', delimiter=",")
    print str(TJM)
    with open('/home/lizq/win7share/TJM.txt', 'r') as ndi:
        print TJM


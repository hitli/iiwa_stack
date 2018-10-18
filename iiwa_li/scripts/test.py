#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import time
import subprocess
import pandas as pd

from numpy.core.multiarray import ndarray

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

def hanshu(a,b):
    print a[0]
    print b

def normalization(x,y,z):
    fenmu = math.sqrt(x**2+y**2+z**2)
    return [i / fenmu for i in [x, y, z]]

if __name__ == '__main__':
    # x = (-32.5000,-20.9000,-1128.1801,-0.6784,0.1082,-0.7137,0.1362)
    # y = (-23.3100,-6.0000,-1151.3300,0.8913,0.1172,0.4375,0.0165)
    # print qc.matrix2quat(qc.quat2matrix(x).dot(qc.quat2matrix(y)))
    # print qc.quat_pose_multipy(x,y)
    # print 0.18456656039890573*0.18456656039890573+-0.032630922568744995*-0.032630922568744995+0.19420782694834174*0.19420782694834174+0.9628882217717862*0.9628882217717862
    # print 0.18452686999999995*0.18452686999999995+0.029733259999999984*0.029733259999999984+0.19421673*0.19421673+0.96279567*0.96279567
    # ndi = np.genfromtxt('/home/lizq/win7share/NDI.txt', delimiter=",")
    # ton = np.loadtxt('/home/lizq/win7share/TON.txt', delimiter=",")
    # tmb = qc.quat2matrix(ndi[0])  # type: ndarray
    # distance = pow(
    #     pow(tmb[0][3] - ton[0][3], 2) + pow(tmb[1][3] - ton[1][3], 2) + pow(tmb[2][3] - ton[2][3], 2),
    #     0.5)
    # # sentence = "ndi下钢针位置：\n" + str(tmb) + "\nndi下穿刺针位置\n" + str(ton) + "针尖距离： %s mm" % distance
    # sentence = "ndi下钢针位置：\n%s\nndi下穿刺针位置\n%s\n针尖距离：%fmm" %(tmb,ton,distance)
    # print sentence

    # p1 = (-41.8100,299.8500,-1634.7100,0.1681,-0.5150,0.2577,0.8000)
    # p2 = (17.5000,192.4200,-1686.9700,0.0,0.0,0.0,1.0)
    # # print qc.point_distance(p1,p2)
    # # print math.acos(qc.matrix2quat(qc.quat2matrix(p2).dot(np.linalg.inv(qc.quat2matrix(p1))))[6]) * 2.0 * 180.0 / np.pi
    # print qc.quat_matrix_multipy(p1,p2)
    # print qc.quat_pose_multipy(p1,p2)
    # print qc.point_distance(qc.quat_matrix_multipy(p1,p2),qc.quat_pose_multipy(p1,p2))

#  四元数算法快的多 0.000236 9.00000000001e-06 0.000227
#                 0.000264 2e-05 0.000244
#0.08944087284948032,1.9035529046321358
    #
    # coscoreid = subprocess.Popen('pgrep roscore', shell=True, stdout=subprocess.PIPE)
    # out, err = coscoreid.communicate()
    # line = 'kill '+out
    # subprocess.call(line, shell=True)
    #
    # tjo = [121.322486525,-410.3412936,776.154925127,0.795024514198,-0.57501980961,0.0924391349392,0.169538422535]
    # ndi = [137.5000,192.4200,-186.9700,0.1686,-0.5152,0.2581,0.7996]
    # rz = [0.0, 0.0, 0.0, 0.0, 0.0, 0.7071, 0.7071]
    # print "tjo"
    # print qc.quat2matrix(tjo)
    # print "tbm"
    # print np.linalg.inv(qc.quat2matrix(ndi))
    # print "rz"
    # print qc.quat2matrix(rz)

    # rz = [0.0, 0.0, 0.0, 0.0, 0.0, 0.7071, 0.7071]
    # with open('/home/lizq/win7share/NDI.txt', 'r') as ndi:
    #     ndi449 = ndi.read().splitlines()[0]
    # print "1"+ndi449

    # with open('/home/lizq/win7share/ndi_data.txt', 'r') as f:
    #     lines = f.read().splitlines()
    # with open('/home/lizq/win7share/ndi_data.txt', 'r') as f:
    #     lines = f.readlines()
    # with open('/home/lizq/win7share/ndi_data.txt', 'w') as f:
    #     n=[]
    #     i=0
    #     for line in lines:
    #         if not "miss" in line:
    #             f.write(line)
    #             n.append(i)
    #         i += 1
    # with open('/home/lizq/win7share/robot_data.txt', 'r') as f:
    #     lines = f.readlines()
    # with open('/home/lizq/win7share/robot_data.txt', 'w') as f:
    #     for i in n:
    #         f.write(lines[i])ndi449 = np.genfromtxt('/home/lizq/win7share/NDI.txt', delimiter=",")[0]
    # subprocess.call("cp /home/lizq/win7share/Rx.txt /home/lizq/win7share/自动标定矩阵保存 ", shell=True)
    # ndi449 = np.genfromtxt('/home/lizq/win7share/NDI.txt', delimiter=",")[1]
    # print math.isnan(ndi449[0])
    # TMC = (137.5000, 192.4200, -1686.9700, 0.1686, -0.5152, 0.2581, 0.7996)
    # sentence = "发送标定向量\n" + str(TMC)
    # client_data = sentence.splitlines()
    # print eval(client_data[1])
    # s = str((137.5000, 192.4200, -1686.9700, 0.1686, -0.5152, 0.2581, 0.7996))
    # print type(eval(s))
    # TBN = np.loadtxt('/home/lizq/win7share/TBN.txt', delimiter=",")
    # TBN_quat = qc.matrix2quat(TBN)
    #
    # with open('/home/lizq/win7share/NDI.txt', 'r') as ndi:
    #     ndi = ndi.read().splitlines()
    #     print eval(ndi[1])
    #
    # np.genfromtxt('/home/lizq/win7share/NDI.txt', delimiter=",")[1].tolist()
    # tjg = np.loadtxt('/home/lizq/win7share/TBN.txt', delimiter=",")
    # print tjg
    # tjn = np.array([[tjg[0][2], -tjg[0][0], -tjg[0][1], tjg[0][3]],
    #                 [tjg[1][2], -tjg[1][0], -tjg[1][1], tjg[1][3]],
    #                 [tjg[2][2], -tjg[2][0], -tjg[2][1], tjg[2][3]],
    #                 [0.0, 0.0, 0.0, 1.0]])
    # print np.linalg.inv(tjg).dot(tjn)
    # print qc.matrix2quat( np.linalg.inv(tjg).dot(tjn))

    # tmn1 = (0, 0, 0)
    # tmn2 = (0, 1, 0)
    # fenmu = math.sqrt((tmn2[0] - tmn1[0]) ** 2 + (tmn2[1] - tmn1[1]) ** 2 + (tmn2[2] - tmn1[2]) ** 2)  # 归一化
    # xx = (tmn2[0] - tmn1[0]) / fenmu
    # xy = (tmn2[1] - tmn1[1]) / fenmu
    # xz = (tmn2[2] - tmn1[2]) / fenmu
    # ndi = np.genfromtxt('/home/lizq/win7share/NDI.txt', delimiter=",")
    # ndi449 = qc.quat2matrix(ndi[0].tolist())
    # y0x = -ndi449[0][0]
    # y0y = -ndi449[1][0]
    # y0z = -ndi449[2][0]  # 穿刺针y方向应取当前tcp**相对于NDI的**y方向附近，可以被ndi所视方向,即被动刚体-x
    # zx, zy, zz = qc.chacheng(xx, xy, xz, y0x, y0y, y0z)  # x叉乘y`得到TCP在视觉空间的z方向，保证x的方向
    # yx, yy, yz = qc.chacheng(zx, zy, zz, xx, xy, xz)  # z叉乘x得到y，使得刚体面向变化较小
    # tmn_jinzhen = np.array([[xx, yx, zx, tmn1[0]],
    #                         [xy, yy, zy, tmn1[1]],
    #                         [xz, yz, zz, tmn1[2]],
    #                         [0.0, 0.0, 0.0, 1.0]])  # 进针点TMN位恣矩阵
    # print tmn_jinzhen
    #
    # a=np.array([[0, 1, 0, 0],
    #           [0, 0, 1, 0],
    #           [1, 0, 0, 0],
    #           [0.0, 0.0, 0.0, 1.0]])  # 进针点TMN位恣矩阵
    # print qc.matrix2quat(a)
    # print type(eval("12.0"))

    # p = np.genfromtxt('/home/lizq/win7share/NDI.txt', delimiter=",")[1]
    # tmn1 = [1.3,2.3,3.3]
    # dx = p[0] - tmn1[0]
    # dy = p[1] - tmn1[1]
    # dz = p[2] - tmn1[2]
    #
    # print dx,dy,dz
    # a=np.array([[0.023253,0.999753,0.007923,683.405138],
    #           [-0.404332,0.016806,-0.914669,-1312.145584],
    #           [-0.914341,0.022574,0.404529,999.771630],
    #           [0.0, 0.0, 0.0, 1.0]])
    # # print qc.matrix2quat(a)
    # # a = qc.quat2matrix(qc.matrix2quat(a))
    # print np.linalg.inv(a)
    # print qc.matrix2quat(np.linalg.inv(a))
    # q = qc.matrix2quat(a)
    # print qc.inv_quat(q)
    # print qc.point_distance(qc.matrix2quat(np.linalg.inv(a)),qc.inv_quat(q))
    #
    # q = (4.6500,198.8300,-1206.4900,0.6439,-0.0888,0.7597,0.0137)
    # a = qc.quat2matrix(q)
    # print np.linalg.inv(a)
    # print qc.matrix2quat(np.linalg.inv(a))
    # print qc.inv_quat(q)
    # print qc.point_distance(qc.inv_quat(q),qc.matrix2quat(np.linalg.inv(a)))
    # ndi = np.genfromtxt('/home/lizq/win7share/NDI.txt', delimiter=",")
    # qc.quat_pose_multipy(ndi[0], ndi[2])
    #
    # ndi = np.genfromtxt('/home/lizq/win7share/NDI.txt', delimiter=",")
    # tmn = list(qc.quat_pose_multipy(ndi[0], ndi[2]))
    # print tmn
    # #
    # # a = (1,1,1,1,1,1,1)
    # # tcp = list(a)
    # # tcp[0,3] *= 1000
    # # print  tcp
    #
    # # tgn = (0., 0., 0., 0.5, -0.5, 0.5, 0.5)
    # # print qc.quat2matrix(tgn)
    # # a = np.array([[0.023253, 0.999753, 0.007923, 683.405138],
    # #               [-0.404332, 0.016806, -0.914669, -1312.145584],
    # #               [-0.914341, 0.022574, 0.404529, 999.771630],
    # #               [0.0, 0.0, 0.0, 1.0]])
    # # b=np.array([[0, 1, 0, -683],
    # #           [0, 0, 1, 1312],
    # #           [1, 0, 0, 999],
    # #           [0.0, 0.0, 0.0, 1.0]])  # 进针点TMN位恣矩阵
    # # print a.dot(b)
    # path =(0,1,2,3,4,5,6)
    # p1 = (path[0:3],0,0,0,1)
    # # print p1
    #
    # ndi = np.genfromtxt('/home/lizq/win7share/NDI.txt', delimiter=",")
    # print ndi[0][0]
    # tgn = np.array([[0., -1., 0., 0.],  # 垂直穿刺针
    #                 [1., 0., 0., 0.],
    #                 [0., 0., 1., 0.],
    #                 [0., 0., 0., 1.]])  # 穿刺针z为钢针z方向，穿刺针y方向为钢针-x,因此穿刺针x方向为钢针y
    # print qc.matrix2quat(tgn)
    # x ,y,z = normalization(1, 2, 3)
    # print x
    # TON = np.loadtxt('/home/lizq/win7share/TON.txt', delimiter=",")
    # print qc.inv_quat(qc.matrix2quat(TON))
    # tgg = np.genfromtxt('/home/lizq/win7share/TGG.txt', delimiter=",")
    # tgg = qc.matrix2quat(tgg)
    # a = (-115.03,-61.601,-170.533,0.918,0.329,-0.038,0.218)
    # print a[3]**2+a[4]**2+a[5]**2+a[6]**2
    # print qc.quat_pose_multipy(a,tgg)
    # tmn = []
    # for i in range(10):
    #     tmn.append((1,2,3))
    # print tmn[1][1]
    # x = [1,2,3]
    # y = [3,4,5]
    # for i in (x, y):
    #     i = (max(i) + min(i)) / 2.0
    # print x
    # TON = np.loadtxt('/home/lizq/win7share/TON.txt', delimiter=",")
    # tno = (-TON[0][3], -TON[1][3], -TON[2][3], 0., 0., 0., 1.)
    # TON_quat = qc.matrix2quat(TON)
    # # too = (-0.83963888104813122, 0.06318205948900868, -0.32905497717041499, 0.0018369972441633557, 0.0034237864364687056, 0.0015511079453492112, 0.99999124855755828)
    # too = (-0.83963888104813122, 0.06318205948900868, -0.32905497717041499, 0.0, 0.5, 0.0, 0.5)
    # tnn = qc.quat_pose_multipy(qc.quat_pose_multipy(tno, too), TON_quat)
    # print np.sqrt(too[0] ** 2 + too[1] ** 2 + too[2] ** 2)
    # print np.sqrt(tnn[0] ** 2 + tnn[1] ** 2 + tnn[2] ** 2)
    # print tnn
    a = []
    a.append(1)
    a =[]
    print a
    print math.sin(3.14)

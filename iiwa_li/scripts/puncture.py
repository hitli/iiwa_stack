#!/usr/bin/env python
# -*- coding: utf-8 -*-


from Tkinter import *
import rospy
import numpy as np
import quaternion_calculate as qc
import math
from geometry_msgs.msg import PoseStamped
from numpy.linalg import inv

class Application(Frame):

    def __init__(self, master=None):
        global PUB
        rospy.init_node('puncture', anonymous=True)
        PUB = rospy.Publisher('/iiwa/command/CartesianPose', PoseStamped, queue_size=10)
        Frame.__init__(self, master)
        self.pack()  # 把Widget加入到父容器中，并实现布局.pack()是最简单的布局，grid()可以实现更复杂的布局。
        self.createWidgets()

    def createWidgets(self):
        self.jinzhendian = Button(self, text='记录进针点', command=self.readjinzhen)
        self.chuancidian = Button(self, text='记录穿刺点', command=self.readchuanci)
        self.button0 = Button(self, text='解算', command=self.cal)
        self.jinzhen = Button(self, text='移至目标点', command=self.movejinzhen)
        self.chuanci = Button(self, text='进针', command=self.movechuanci)
        self.jinzhendian.pack()
        self.chuancidian.pack()
        self.button0.pack()
        self.jinzhen.pack()
        self.chuanci.pack()

    def readjinzhen(self):
        global P1
        P1 = np.loadtxt('/home/lizq/win7share/NDI.txt', delimiter=",").tolist()  # 屏蔽449被动刚体

    def readchuanci(self):
        global P2
        P2 = np.loadtxt('/home/lizq/win7share/NDI.txt', delimiter=",").tolist()

    def cal(self):
        global TJO_jinzhen
        global TJO_chuanci
        tjm = np.loadtxt('/home/lizq/win7share/TJM.txt', delimiter=",")  # mm
        ton = np.loadtxt('/home/lizq/win7share/TON.txt', delimiter=",")  # mm
        tgg = np.loadtxt('/home/lizq/win7share/TGG.txt', delimiter=",")  # mm
        tno = inv(ton)
        tmg1 = qc.quat2matrix(P1).dot(tgg) #矫正钢针针尖位置
        tmg2 = qc.quat2matrix(P2).dot(tgg)
        x = math.sqrt((tmg2[0][3] - tmg1[0][3]) ** 2 + (tmg2[1][3] - tmg1[1][3]) ** 2 + (tmg2[2][3] - tmg1[2][3]) ** 2)  # 归一化
        xx = (tmg2[0][3] - tmg1[0][3]) / x
        xy = (tmg2[1][3] - tmg1[1][3]) / x
        xz = (tmg2[2][3] - tmg1[2][3]) / x  # TCP入针，即视觉空间的TCP（同穿刺针）的x方向

        y0x = -tmg1[0][0]
        y0y = -tmg1[1][0]
        y0z = -tmg1[2][0]  # y`方向为钢针-x方向
        
        zx,zy,zz = self.chacheng(xx,xy,xz,y0x,y0y,y0z)  # x叉乘y`得到TCP在视觉空间的z方向，保证x的方向

        yx,yy,yz = self.chacheng(zx,zy,zz,xx,xy,xz)  # z叉乘x得到y，使得刚体面向变化较小

        tmn_jinzhen = np.array([[xx, yx, zx, tmg1[0][3]],
                                [xy, yy, zy, tmg1[1][3]],
                                [xz, yz, zz, tmg1[2][3]],
                                [0.0, 0.0, 0.0, 1.0]])  # 进针点TMN位恣矩阵

        tmn_chuanci = np.array([[xx, yx, zx, tmg2[0][3]],
                                [xy, yy, zy, tmg2[1][3]],
                                [xz, yz, zz, tmg2[2][3]],
                                [0.0, 0.0, 0.0, 1.0]])  # 穿刺点TMN位恣矩阵
        TJO_jinzhen = tjm.dot(tmn_jinzhen).dot(tno)
        TJO_jinzhen[0:3][:, 3] /= 1000
        TJO_chuanci = tjm.dot(tmn_chuanci).dot(tno)
        TJO_chuanci[0:3][:, 3] /= 1000


    def chacheng(self,x1,y1,z1,x2,y2,z2):
        return y1*z2-y2*z1,z1*x2-z2*x1,x1*y2-x2*y1


    def movejinzhen(self):
        PUB.publish(qc.get_command_pose(qc.matrix2quat(TJO_jinzhen))


    def movechuanci(self):
        PUB.publish(qc.get_command_pose(qc.matrix2quat(TJO_chuanci)))


app = Application()
# 设置窗口标题:
app.master.title('puncture')
# 主消息循环:
app.mainloop()

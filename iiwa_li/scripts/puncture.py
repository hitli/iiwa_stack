#!/usr/bin/env python
# -*- coding: utf-8 -*-


from Tkinter import *
import rospy
import numpy as np
import quaternion_calculate as qc
import math
from geometry_msgs.msg import PoseStamped


class Application(Frame):
    
    def __init__(self, master=None):
        # rospy.init_node('puncture', anonymous=True)
        # pub = rospy.Publisher('/iiwa/command/CartesianPose', PoseStamped, queue_size=10)
        Frame.__init__(self, master)
        self.pack() #把Widget加入到父容器中，并实现布局.pack()是最简单的布局，grid()可以实现更复杂的布局。
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
        global p1
        p1 = np.loadtxt('/home/lizq/win7share/NDI.txt', delimiter=",").tolist()

    def readchuanci(self):
        global p2
        p2 = np.loadtxt('/home/lizq/win7share/NDI.txt', delimiter=",").tolist()

    def cal(self):
        global p1#视觉空间下
        global p2
        x = math.sqrt((p2[0] - p1[0]) ** 2 + (p2[1] - p1[1]) ** 2 + (p2[2] - p1[2]) ** 2)
        xx = (p2[0] - p1[0]) / x
        xy = (p2[1] - p1[1]) / x
        xz = (p2[2] - p1[2]) / x #归一化TCPx方向

        y = qc.quat2matrix(p1)
        yx = -y[0][0]
        yy = -y[1][0]
        yz = -y[2][0] #TCP y方向为钢针-x方向
        
        zx = xy*yz-yy*xz
        zy = xz*yx-yz*xx
        zz = xx*yy-yx*xy #x叉乘y得到TCP在视觉空间的z方向
        
        




    
    def movejinzhen(self):
        pass

    def movechuanci(self):
        pass


app = Application()
# 设置窗口标题:
app.master.title('puncture')
# 主消息循环:
app.mainloop()

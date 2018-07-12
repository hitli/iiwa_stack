#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import math
import quaternion_calculate as qc
import time

tgg = np.loadtxt('/home/lizq/win7share/TGG.txt', delimiter=",")  # mm 需要有话
while True:
    try:
        ndi = np.genfromtxt('/home/lizq/win7share/NDI.txt', delimiter=",")
        if math.isnan(ndi[1][0]):
            print "等待钢针"
        else:
            tmg = qc.quat2matrix(ndi[1].tolist())  # 被动刚体位姿
            tmg = tmg.dot(tgg)  # 更正钢针位姿
            print qc.matrix2quat(tmg)
    except:
        pass
    time.sleep(0.5)
#!/usr/bin/env python
# -*- coding: utf-8 -*-
import quaternion_calculate as qc
import numpy as np

tjm = np.loadtxt('/home/lizq/win7share/TJM.txt', delimiter=",")  # mm
tjm_quat = qc.matrix2quat(tjm)
tmg = (204.2589,140.4374,-1461.4745,0.0268,-0.6993,-0.0937,0.7081)
tjg = qc.quat_pose_multipy(tjm_quat,tmg)
print tjg
p2 = np.zeros((8,10,7))
for i in range(8):
    for j in range(10):
        p2[i][j] = tjg
        p2[i][j][0] += 3*(7-i)
        p2[i][j][1] += 3 * (9 - j)
print p2[7][9]
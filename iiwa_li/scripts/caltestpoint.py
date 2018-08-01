#!/usr/bin/env python
# -*- coding: utf-8 -*-
import quaternion_calculate as qc
import numpy as np
import copy

tjm = np.loadtxt('/home/lizq/win7share/TJM.txt', delimiter=",")  # mm
tjm_quat = qc.matrix2quat(tjm)
# tmg = (204.2589,140.4374,-1461.4745,0.0268,-0.6993,-0.0937,0.7081)
# tmg = (205.6476,138.6107,-1459.6112,-0.1418,-0.6499,-0.3187,0.6753)
# tmg = (201.7759,145.6441,-1461.7288,0.0774,-0.5906,-0.0728,0.7999) # 泥实验
# tmg = (204.5578030021104, 138.97664726986795, -1470.5886595634481, 0.0709516475635896, -0.5880581420528788, -0.07524077508540633, 0.8021797248982772)
# tmg = (211.3658865331679, 145.71588744753421, -1467.2212266498939, 0.07512071845855048, -0.5903339289788885, -0.07389869633323265, 0.8002510310114375)  # rou
# tmg = (214.1174,144.7852,-1451.8646,0.1087,-0.5781,0.8023,0.1016)  # 换盛具
# tmg = (236.7951,356.0809,-1411.2412,0.2308,-0.6254,0.6982,0.2608)  # 斜向丢点变更位置
tmg = (182.4626,142.9635,-1509.2392,0.3461,-0.4871,0.5876,0.5457)  # 粗针橡皮泥
tjg = qc.quat_pose_multipy(tjm_quat,tmg)
print tjg

# print qc.quat_pose_multipy(qc.matrix2quat(np.linalg.inv(tjm)),tjg)
i = 20.
p2 = np.zeros((8,i,7))
p1 = np.zeros((8,20,7))
for i in range(8):
    for j in range(20):
        p2[i][j] = copy.deepcopy(tjg)
        p2[i][j][0] += 4*(7-i)
        p2[i][j][1] += 3 * (20 - j)
for i in (1,3,5,7):
    for j in range(20):
        p1[i][j] = copy.deepcopy(p2[i][j])
        p1[i][j][2] += 80
for i in (0,2,4,6):
    for j in range(20):
        p1[i][j] = copy.deepcopy(p2[i][j])
        p1[i][j][2] += 80
        p1[i][j][1] += 20
tmp1 = np.zeros((8,20,7))
tmp2 = np.zeros((8,20,7))
path = np.zeros((8,20,6))
for i in range(8):
    for j in range(20):
        tmp1[i][j] = qc.quat_pose_multipy(qc.matrix2quat(np.linalg.inv(tjm)),p1[i][j])
        tmp2[i][j] = qc.quat_pose_multipy(qc.matrix2quat(np.linalg.inv(tjm)),p2[i][j])
        path[i][j] = [tmp1[i][j][0],tmp1[i][j][1],tmp1[i][j][2],tmp2[i][j][0],tmp2[i][j][1],tmp2[i][j][2]]
for i in range(20):
    print ','.join(str(k) for k in path[4][i])

# g = (901.87715436375697, -54.70624009748667, 231.15945922773415, -0.012568588918144052, -0.05926193790309206, -0.7876875889199433, 0.613089157912667)
# print qc.quat_pose_multipy(qc.matrix2quat(np.linalg.inv(tjm)),g)
#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import quaternion_calculate as qc
import math

def zhenjian():

    tgg = np.loadtxt('/home/lizq/win7share/TGG.txt', delimiter=",")  # mm
    tbn = np.loadtxt('/home/lizq/win7share/TBN.txt', delimiter=",")
    tbo = np.loadtxt('/home/lizq/win7share/TBO.txt', delimiter=",")
    ton = np.loadtxt('/home/lizq/win7share/TON.txt', delimiter=",")
    tmg = qc.quat2matrix(np.loadtxt('/home/lizq/win7share/NDI.txt', delimiter=",", skiprows=1).tolist())  # mm
    tmb = qc.quat2matrix(np.loadtxt('/home/lizq/win7share/NDI.txt', delimiter=",")[0].tolist())
    tmg = tmg.dot(tgg)
    tmn = tmb.dot(tbn)
    print qc.matrix2quat(tmg)  # math.sqrt((qc.matrix2quat(tmg)[0:3]-qc.matrix2quat(tmn)[0:3])*((qc.matrix2quat(tmg)[0:3]-qc.matrix2quat(tmn)[0:3]).T))
    print qc.matrix2quat(tmn)
    tbn = tbo.dot(ton)
    print tbn


if __name__ == '__main__':
    zhenjian()
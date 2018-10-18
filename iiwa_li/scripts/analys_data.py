#!/usr/bin/env python
# -*- coding: utf-8 -*-

import pandas as pd
import matplotlib.pyplot as plt
import time

ati = pd.read_csv('/home/lizq/实验数据整理/1018多段穿肉/ati_v60_step30_2mm_pork_3.csv',header=7,names=['Fx','Fy','Fz','Tx','Ty','Tz'],usecols=[3,4,5,6,7,8])  # 十万行 0.064s
# ati = pd.read_csv('/home/lizq/桌面/ati_test3.csv',header=7,usecols=[3,4,5,6,7,8])  # 五百行 0.0036秒
# ati = pd.read_csv('/home/lizq/桌面/nditest_000.csv',header=0)
# print ati.columns
# 原点相交
ax = plt.gca()
ax.xaxis.set_ticks_position('bottom')
ax.spines['bottom'].set_position(('data',0))
ax.yaxis.set_ticks_position('left')
ax.spines['left'].set_position(('data',0))
# 去掉边框
ax.spines['top'].set_color('none')
ax.spines['right'].set_color('none')
plt.plot(ati.index,-ati.Fx/1000000.)
# plt.scatter(ati.index,ati.Fx/1000000.)
plt.savefig('aa.png')
plt.show()
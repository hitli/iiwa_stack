#!/usr/bin/env python
# -*- coding: utf-8 -*-

import pandas as pd
import matplotlib.pyplot as plt
import time

start = time.time()
ndi = pd.read_csv('/home/lizq/桌面/nditest_014.csv',skiprows = 1,names = ['449rw','449rx','449ry','449rz','449x','449y','449z','340rw','340rx','340ry','340rz','340x','340y','340z','339rw','339rx','339ry','339rz','339x','339y','339z'],usecols=[5,6,7,8,9,10,11,98,99,100,101,102,103,104,191,192,193,194,195,196,197])
# ati = pd.read_csv('/home/lizq/实验数据整理/0730粗针穿刺/ni_v90_xie/ati_ni_v90_xie1.csv',header=7,names=['Fx','Fy','Fz','Tx','Ty','Tz'],usecols=[3,4,5,6,7,8])  # 十万行 0.064s
end = time.time()
print ndi.index
print start-end
# print ndi
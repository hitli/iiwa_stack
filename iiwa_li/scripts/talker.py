#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


import rospy
import quaternion_calculate as qc
from geometry_msgs.msg import PoseStamped
import matlab.engine

TCP_position = tuple()
start_position = tuple()


# 获得标定开始点TCP位置
def get_TCP_position():
    print('running')
    global start_position
    while TCP_position == ():
        print('loading TCP position')
        rospy.Subscriber('/iiwa/state/CartesianPose', PoseStamped, save_start_position)
    else:
        start_position = TCP_position
        print('TCP position get :', start_position)
        with open('/home/lizq/win7share/TCP.txt', 'w') as f:  # 记录TCP位置
            string = ','.join(str(i) for i in start_position)
            f.write(string)


def save_start_position(start):
    global TCP_position
    TCP_position = (start.pose.position.x, start.pose.position.y, start.pose.position.z, start.pose.orientation.x,
                    start.pose.orientation.y, start.pose.orientation.z, start.pose.orientation.w)


def biaoding(step=10, lengh=0.02, rad=0.15):
    pub = rospy.Publisher('/iiwa/command/CartesianPose', PoseStamped, queue_size=10)
    rospy.sleep(1)  # 1s
    # global TCP_position
    for axs in ('rx', 'ry', 'rz', 'dx', 'dy', 'dz'):  # x,y,z旋转,x,y,z平移
        for n in range(1, step + 1):
            calibrate_point = qc.turn_TCP_axs_rad_len(start_position, axs, rad * n, lengh * n)
            # 为命令赋值
            command_point = qc.get_command_pose(calibrate_point,n)
            # print command_point
            rospy.loginfo(command_point)
            pub.publish(command_point)
            rate = rospy.Rate(0.7)  # 0.5hz
            rate.sleep()
            with open('/home/lizq/win7share/NDI.txt', 'r') as ndi:
                write_to_txt(axs, n, ndi.read())
        command_point = qc.get_command_pose(start_position)
        rospy.loginfo(command_point)
        pub.publish(command_point)
        rate = rospy.Rate(0.3)  # 0.3hz
        rate.sleep()
    print '加载最小二乘法'
    eng = matlab.engine.start_matlab()
    eng.MinTwoSolveTJM(nargout=0)





def write_to_txt(axs, n, str):  #写入txt文件
    if axs == 'rx':
        if n == 1:
            with open('/home/lizq/win7share/Rx.txt', 'w') as f:  # 覆盖从头写入
                f.write(str)
        else:
            with open('/home/lizq/win7share/Rx.txt', 'a') as f:  # 从末尾写入
                f.write('\r\n')
                f.write(str)

    if axs == 'ry':
        if n == 1:
            with open('/home/lizq/win7share/Ry.txt', 'w') as f:  # 覆盖从头写入
                f.write(str)
        else:
            with open('/home/lizq/win7share/Ry.txt', 'a') as f:  # 从末尾写入
                f.write('\r\n')
                f.write(str)

    if axs == 'rz':
        if n == 1:
            with open('/home/lizq/win7share/Rz.txt', 'w') as f:  # 覆盖从头写入
                f.write(str)
        else:
            with open('/home/lizq/win7share/Rz.txt', 'a') as f:  # 从末尾写入
                f.write('\r\n')
                f.write(str)

    if axs == 'dx':
        if n == 1:
            with open('/home/lizq/win7share/Dx.txt', 'w') as f:  # 覆盖从头写入
                f.write(str)
        else:
            with open('/home/lizq/win7share/Dx.txt', 'a') as f:  # 从末尾写入
                f.write('\r\n')
                f.write(str)

    if axs == 'dy':
        if n == 1:
            with open('/home/lizq/win7share/Dy.txt', 'w') as f:  # 覆盖从头写入
                f.write(str)
        else:
            with open('/home/lizq/win7share/Dy.txt', 'a') as f:  # 从末尾写入
                f.write('\r\n')
                f.write(str)

    if axs == 'dz':
        if n == 1:
            with open('/home/lizq/win7share/Dz.txt', 'w') as f:  # 覆盖从头写入
                f.write(str)
        else:
            with open('/home/lizq/win7share/Dz.txt', 'a') as f:  # 从末尾写入
                f.write('\r\n')
                f.write(str)


if __name__ == '__main__':
    rospy.init_node('calibrate_commander', anonymous=True)
    get_TCP_position()
    biaoding(10, 0.015, 0.15)

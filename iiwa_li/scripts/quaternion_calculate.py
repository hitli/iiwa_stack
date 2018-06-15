#!/usr/bin/env python
# -*- coding: utf-8 -*-


import numpy as np
import math
from geometry_msgs.msg import PoseStamped
from iiwa_msgs.msg import JointPosition


def quat2matrix((dx, dy, dz, x, y, z, w)):
    #输入tuple
    matrix = np.zeros((4,4))
    # 规范化处理，保证模为1
    x = x / math.sqrt(x * x + y * y + z * z + w * w)
    y = y / math.sqrt(x * x + y * y + z * z + w * w)
    z = z / math.sqrt(x * x + y * y + z * z + w * w)

    matrix[0] = [2*(w*w+x*x)-1 , 2*(x*y-w*z) , 2*(x*z+w*y) , dx]
    matrix[1] = [2*(x*y+w*z) , 2*(w*w+y*y)-1 , 2*(y*z-w*x) , dy]
    matrix[2] = [2*(x*z-w*y) , 2*(y*z+w*x) , 2*(w*w+z*z)-1 , dz]
    matrix[3] = [0,0,0,1]
    return matrix


def matrix2quat(matrix):
    #输入numpy矩阵
    mw=1+matrix[0][0]+matrix[1][1]+matrix[2][2]
    mx=1+matrix[0][0]-matrix[1][1]-matrix[2][2]
    my=1-matrix[0][0]+matrix[1][1]-matrix[2][2]
    mz=1-matrix[0][0]-matrix[1][1]+matrix[2][2]
    # if abs(mw)<1e-7:
    #     mw=0
    # if abs(mx)<1e-7:
    #     mx=0
    # if abs(my)<1e-7:
    #     my=0
    # if abs(mz)<1e-7:
    #     mz=0
    # try:
    #     w = 0.5*math.sqrt(mw)
    #     x = 0.5*math.sqrt(mx)
    #     y = 0.5*math.sqrt(my)
    #     z = 0.5*math.sqrt(mz)
    # except BaseException,e:
    #     print e
    w = 0.5 * math.sqrt(abs(mw))
    x = 0.5 * math.sqrt(abs(mx))
    y = 0.5 * math.sqrt(abs(my))
    z = 0.5 * math.sqrt(abs(mz))

    #规范化处理，保证模为1
    x = x / math.sqrt(x * x + y * y + z * z + w * w)
    y = y / math.sqrt(x * x + y * y + z * z + w * w)
    z = z / math.sqrt(x * x + y * y + z * z + w * w)

    if matrix[2][1]-matrix[1][2] < 0:
        x = -x
    if matrix[0][2]-matrix[2][0] < 0:
        y = -y
    if matrix[1][0]-matrix[0][1] < 0:
        z = -z
    #返回tuple
    return matrix[0][3], matrix[1][3], matrix[2][3], x, y, z, w


def quatmultipy((x1,y1,z1,w1),(x2,y2,z2,w2)):
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    return x,y,z,w
#
# # 不对!!
# def quat_pose_multipy((dx1,dy1,dz1,x1,y1,z1,w1),(dx2,dy2,dz2,x2,y2,z2,w2)):
#     q = (x1,y1,z1,w1)
#     qt = (-x1,-y1,-z1,w1)
#     qw = (dx2,dy2,dz2,0)
#     qw2 = quatmultipy(quatmultipy(qt,qw),q)
#     dx = dx1 + qw2[0]
#     dy = dy1 + qw2[1]
#     dz = dz1 + qw2[2]
#     x,y,z,w = quatmultipy((x1,y1,z1,w1),(x2,y2,z2,w2))
#     return dx,dy,dz,x,y,z,w


def quat_matrix_multipy(q1,q2):
    quat = matrix2quat(quat2matrix(q1).dot(quat2matrix(q2)))
    return quat


def turn_TCP_axs_rad_len(position,axs,rad,len):  # 自动标定用
    if axs=='rx':
        Tmat=quat2matrix((0,0,0,math.sin(rad/2.0),0,0,math.cos(rad/2.0)))  # 对x轴正转
    elif axs=='ry':
        Tmat=quat2matrix((0,0,0,0,math.sin(rad/2.0),0,math.cos(rad/2.0)))
    elif axs=='rz':
        Tmat=quat2matrix((0,0,0,0,0,math.sin(rad/2.0),math.cos(rad/2.0)))
    elif axs=='dx':
        Tmat=quat2matrix((len,0,0,0,0,0,1))
    elif axs == 'dy':
        Tmat = quat2matrix((0, len, 0, 0, 0, 0, 1))
    elif axs == 'dz':
        Tmat = quat2matrix((0, 0, len, 0, 0, 0, 1))
    else:
        print ('输入轴错误')
    # print 'Tmat',Tmat
    # print 'quat2matrix(position)',quat2matrix(position)
    # print 'quat2matrix(position)*Tmat',np.dot(quat2matrix(position),Tmat)
    return matrix2quat(np.dot(quat2matrix(position),Tmat))  # 右乘,相对末端运动


def point_distance(p1,p2):  # 计算两点位姿差
    d1 = np.mat(p1[0:3])
    d2 = np.mat(p2[0:3])
    distance = np.sqrt((d1 - d2) * (d1 - d2).T)  # mm

    # 计算四元数的除法
    q1 = p1[3:7]
    q2 = p2[3:7]
    try:
        q1[0:3] = -q1[0:3]  # 求逆
        q = quatmultipy(q1,q2)
        degree = math.acos(q[3])*2.0*180.0/np.pi
    except:
        q2[0:3] = -q2[0:3]
        q = quatmultipy(q2,q1)
        degree = math.acos(q[3]) * 2.0 * 180.0 / np.pi
    return distance,degree


def quat2angle(quat):#没用
    q1=2*(quat[3]*quat[4]+ quat[6]*quat[5])
    q2=quat[6]**2 + quat[3]**2 - quat[4]**2 - quat[5]**2
    q3=-2*(quat[3]*quat[5]- quat[6]*quat[4])
    q4=2*(quat[4]*quat[5]+ quat[6]*quat[3])
    q5=quat[6]**2 - quat[3]**2 - quat[4]**2 + quat[5]**2
    return (quat[0],quat[1],quat[2],math.atan2(q1,q2),math.asin(q3),math.atan2(q4,q5))


def get_command_pose(calibrate_point,n=0):  # 输入米
    command_point = PoseStamped()
    command_point.header.seq = n
    command_point.pose.position.x = calibrate_point[0]
    command_point.pose.position.y = calibrate_point[1]
    command_point.pose.position.z = calibrate_point[2]
    command_point.pose.orientation.x = calibrate_point[3]
    command_point.pose.orientation.y = calibrate_point[4]
    command_point.pose.orientation.z = calibrate_point[5]
    command_point.pose.orientation.w = calibrate_point[6]
    return command_point


def get_command_joint(calibrate_point):  # 输入弧度
    command_point=JointPosition()
    command_point.position.a1 = calibrate_point[0]
    command_point.position.a2 = calibrate_point[1]
    command_point.position.a3 = calibrate_point[2]
    command_point.position.a4 = calibrate_point[3]
    command_point.position.a5 = calibrate_point[4]
    command_point.position.a6 = calibrate_point[5]
    command_point.position.a7 = calibrate_point[6]
    return command_point


def write_to_txt(axs, n, str):  # 写入txt文件
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
    print (quat2matrix((1,2,3,1,0,0,0)))
    # print matrix2quat(quat2matrix((1,2,3,1,0,0,0)))
    # print(turn_TCP_axs_rad((1, 2, 3, 1, 0, 0, 0), 'rx', 0))
    # eng=matlab.engine.start_matlab()
    # eng.MinTwoSolveTJM(nargout=0)
    # q=[1,0,1,0]
    # print eng.quat2angle(q,nargout=3)
    # print quat2angle((0,0,0,0,0,0,1))
    # print ','.join(str(i) for i in quat2angle((0.0,0,0,0,0,0,1)))
    print quat2matrix((0.3196184910824411, -0.24608946005373455, 0.9227790288091682, -0.16047115123781783, 0.06647591727366217, -0.37690016065474524, 0.9098221063613892))
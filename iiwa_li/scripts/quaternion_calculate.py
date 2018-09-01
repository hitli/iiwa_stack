#!/usr/bin/env python
# -*- coding: utf-8 -*-


import numpy as np
import math
from geometry_msgs.msg import PoseStamped
from iiwa_msgs.msg import JointPosition


def chacheng(x1, y1, z1, x2, y2, z2):
    return y1 * z2 - y2 * z1, z1 * x2 - z2 * x1, x1 * y2 - x2 * y1


def normalization(x,y,z):
    fenmu = math.sqrt(x**2+y**2+z**2)
    return [i / fenmu for i in [x, y, z]]


def normalization_quat((x,y,z,rx,ry,rz,rw)):
    fenmu = math.sqrt(rx**2+ry**2+rz**2+rw**2)
    rx /= fenmu
    ry /= fenmu
    rz /= fenmu
    rw /= fenmu
    return (x,y,z,rx,ry,rz,rw)


def quat2matrix((dx, dy, dz, x, y, z, w)):
    #输入tuple
    matrix = np.zeros((4,4))
    # 规范化处理，保证模为1
    fenmu = math.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = [i / fenmu for i in [x, y, z, w]]
    matrix[0] = [2*(w*w+x*x)-1 , 2*(x*y-w*z) , 2*(x*z+w*y) , dx]
    matrix[1] = [2*(x*y+w*z) , 2*(w*w+y*y)-1 , 2*(y*z-w*x) , dy]
    matrix[2] = [2*(x*z-w*y) , 2*(y*z+w*x) , 2*(w*w+z*z)-1 , dz]
    matrix[3] = [0,0,0,1]
    return matrix


def matrix2quat(matrix):
    # 输入numpy矩阵
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

    # 规范化处理，保证模为1
    fenmu = math.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = [i / fenmu for i in [x, y, z, w]]

    if matrix[2][1]-matrix[1][2] < 0:
        x = -x
    if matrix[0][2]-matrix[2][0] < 0:
        y = -y
    if matrix[1][0]-matrix[0][1] < 0:
        z = -z
    # 返回tuple
    return matrix[0][3], matrix[1][3], matrix[2][3], x, y, z, w


# def quatmultipy((x1,y1,z1,w1),(x2,y2,z2,w2)):
#     w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
#     x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
#     y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
#     z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
#     return x,y,z,w


def point_distance(p1,p2):  # 计算两点位姿差
    fenmu = math.sqrt(p1[3] * p1[3] + p1[4] * p1[4] + p1[5] * p1[5] + p1[6] * p1[6])
    x1 = p1[3]/fenmu
    y1 = p1[4]/fenmu
    z1 = p1[5]/fenmu
    w1 = p1[6]/fenmu
    fenmu = math.sqrt(p2[3] * p2[3] + p2[4] * p2[4] + p2[5] * p2[5] + p2[6] * p2[6])
    x2 = p2[3] / fenmu
    y2 = p2[4] / fenmu
    z2 = p2[5] / fenmu
    w2 = p2[6] / fenmu
    distance = np.sqrt(np.square(p1[0]-p2[0])+np.square(p1[1]-p2[1])+np.square(p1[2]-p2[2]))
    # 计算四元数的除法,即乘逆
    # w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    # x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    # y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    # z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    w = w1 * w2 + x1 * x2 + y1 * y2 + z1 * z2
    if w>1:
        w=w-1
    degree = math.acos(w)*2.0*180.0/np.pi
    return distance,degree


def quat_pose_multipy((dx1,dy1,dz1,x1,y1,z1,w1),(dx2,dy2,dz2,x2,y2,z2,w2)):
    fenmu = math.sqrt(x1 * x1 + y1 * y1 + z1 * z1 + w1 * w1)
    x1, y1, z1, w1 = [i / fenmu for i in [x1, y1, z1, w1]]
    fenmu = math.sqrt(x2 * x2 + y2 * y2 + z2 * z2 + w2 * w2)
    x2, y2, z2, w2 = [i / fenmu for i in [x2, y2, z2, w2]]
    # q = (x1,y1,z1,w1)
    # qt = (-x1,-y1,-z1,w1)
    # qw = (dx2,dy2,dz2,0)
    # qw2 = quatmultipy(quatmultipy(q,qw),qt)
    # 坐标旋转公式展开
    qx2 = (1-2*y1*y1-2*z1*z1)*dx2+(2*x1*y1-2*z1*w1)*dy2+(2*x1*z1+2*y1*w1)*dz2
    qy2 = (2*x1*y1+2*w1*z1)*dx2+(1-2*x1*x1-2*z1*z1)*dy2+(2*y1*z1-2*x1*w1)*dz2
    qz2 = (2*x1*z1-2*y1*w1)*dx2+(2*y1*z1+2*x1*w1)*dy2+(1-2*x1*x1-2*y1*y1)*dz2
    qw2 = 0
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    fenmu = math.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = [i / fenmu for i in [x, y, z, w]]
    return dx1 + qx2,dy1 + qy2,dz1 + qz2,x,y,z,w


def inv_quat((dx, dy, dz, x, y, z, w)):
    fenmu = math.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = [i / fenmu for i in [x, y, z, w]]
    qx = (1-2*y*y-2*z*z)*dx+(2*x*y+2*z*w)*dy+(2*x*z-2*y*w)*dz
    qy = (2*x*y-2*w*z)*dx+(1-2*x*x-2*z*z)*dy+(2*y*z+2*x*w)*dz
    qz = (2*x*z+2*y*w)*dx+(2*y*z-2*x*w)*dy+(1-2*x*x-2*y*y)*dz
    return -qx,-qy,-qz,-x,-y,-z,w


def turn_TCP_axs_rad_len(position,axs,rad,len):  # 自动标定用
    if axs=='rx':
        Tmat=quat2matrix((0,0,0,-math.sin(rad/2.0),0,0,math.cos(rad/2.0)))  # 对x轴正转
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
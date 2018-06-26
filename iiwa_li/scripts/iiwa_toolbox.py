# -*- coding: utf-8 -*-

from mainwindow import Ui_MainWindow
from PyQt5 import QtWidgets, QtCore, QtGui
import sys
import copy
import rospy
import quaternion_calculate as qc
import numpy as np
import math
from geometry_msgs.msg import PoseStamped
from iiwa_msgs.msg import JointPosition
import subprocess
import matlab.engine
import socket
import traceback


class Mywindow(QtWidgets.QMainWindow, Ui_MainWindow):
    def __init__(self):

        # 全局变量
        self.joint_position = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)  # a1,a2,a3,a4,a5,a6,a7 tuple 弧度
        self.tcp_pose = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)  # x,y,z,rx,ry,rz,rw tuple 米
        self.manual_calibrate_count = 1

        super(Mywindow, self).__init__()
        self.setupUi(self)

        self.ndi_error_button.setToolTip("通过角关节位置得到末端位姿TJO,与TJM*TMB*TBO=TJO比较,查看标定误差")
        self.calibrate_error_button.setToolTip("记录449位置,命令末端运动到附近再返回记录位置,比较ndi数据距离,确定重定位精度")
        self.manual_calibrate_button.setToolTip("初始化ndi,robot_data文件,需要可以看到449")
        try:
            self.TJM = np.loadtxt('/home/lizq/win7share/TJM.txt', delimiter=",")  # mm
        except IOError:
            QtWidgets.QMessageBox.information(self, "提示", "未找到TJM标定矩阵，穿刺及跟随前请先执行标定程序")

        # 启动roscore
        # subprocess.Popen('roscore')
        # 启动matlab核心
        self.matlab_eng = matlab.engine.start_matlab()
        print "加载matlab核心"

        #  启动节点
        rospy.init_node('iiwa_toolbox', anonymous=True)
        self.pose_pub = rospy.Publisher('/iiwa/command/CartesianPose', PoseStamped, queue_size=30)
        self.joint_pub = rospy.Publisher('/iiwa/command/JointPosition', JointPosition, queue_size=30)
        rospy.Subscriber('/iiwa/state/JointPosition', JointPosition, self.get_joint_position)
        rospy.Subscriber('/iiwa/state/CartesianPose', PoseStamped, self.get_tcp_pose)

        #  启动定时器
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.read_state)
        self.timer.start(300)  # 300ms

        # 线程,信号槽连接
        self.follow_thread = Follow_Thread()
        self.follow_thread.settext_signal[str].connect(self.textEdit_calibrate_settext_slot)  # 线程.信号.connect(槽)
        self.calibrate_thread = Calibrate_Thread()
        self.calibrate_thread.append_signal[str].connect(self.textEdit_calibrate_append_slot)
        self.calibrate_thread.settext_signal[str].connect(self.textEdit_calibrate_settext_slot)
        self.calibrate_error_thread = Calibrate_Error_Thread()
        self.calibrate_error_thread.append_signal[str].connect(self.textEdit_calibrate_append_slot)
        self.calibrate_error_thread.settext_signal[str].connect(self.textEdit_calibrate_settext_slot)
        self.follow_pose_thread = Follow_Pose_Thread()
        self.server_thread = Server_Thread()
        self.server_thread.settext_signal[str].connect(self.textEdit_ct_settext_slot)
        self.server_thread.append_signal[str].connect(self.textEdit_ct_append_slot)

    def closeEvent(self, event):  # 改写关闭事件，添加对话框及关闭roscore
        reply = QtWidgets.QMessageBox.question(self, 'Message',"Are you sure to quit?", QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No, QtWidgets.QMessageBox.Yes)
        if reply == QtWidgets.QMessageBox.Yes:
            # coscoreid = subprocess.Popen('pgrep roscore', shell=True, stdout=subprocess.PIPE)
            # out, err = coscoreid.communicate()
            # line = 'kill ' + out
            # subprocess.call(line, shell=True)
            event.accept()
        else:
            event.ignore()

    def read_state(self):
        # ','.join(str(i) for i in self.tcp_pose)将数组每一位以,间隔转为字符串
        tcp_pose_output = list(self.tcp_pose)
        for i in range(3):
            tcp_pose_output[i] = round(tcp_pose_output[i] * 1000, 3)  # 显示毫米
        for i in range(3, 7):
            tcp_pose_output[i] = round(tcp_pose_output[i], 3)
        self.lineEdit_pose_out.setText(','.join(str(i) for i in tcp_pose_output))

        joint_position_output = list(self.joint_position)  # 数组化方便操作,tuple不让运算
        for i in range(7):
            joint_position_output[i] = round(joint_position_output[i] * 180.0 / math.pi, 2)  # round(x,2)保留两位小数,显示角度
        self.lineEdit_joint_out.setText(','.join(str(i) for i in joint_position_output))
        self.progressBar_1.setProperty("value", joint_position_output[0])
        self.progressBar_2.setProperty("value", joint_position_output[1])
        self.progressBar_3.setProperty("value", joint_position_output[2])
        self.progressBar_4.setProperty("value", joint_position_output[3])
        self.progressBar_5.setProperty("value", joint_position_output[4])
        self.progressBar_6.setProperty("value", joint_position_output[5])
        self.progressBar_7.setProperty("value", joint_position_output[6])
        try:
            with open('/home/lizq/win7share/NDI.txt', 'r') as ndi:
                ndi = ndi.read().splitlines()
                self.lineEdit_ndi449.setText(ndi[0])
                self.lineEdit_ndi340.setText(ndi[1])
                self.lineEdit_ndi339.setText(ndi[2])
        except IndexError,e:
            print e,ndi

    def get_tcp_pose(self, tcp):
        self.tcp_pose = (
            tcp.pose.position.x, tcp.pose.position.y, tcp.pose.position.z, tcp.pose.orientation.x,
            tcp.pose.orientation.y,
            tcp.pose.orientation.z, tcp.pose.orientation.w)

    def get_joint_position(self, joint):
        self.joint_position = (
            joint.position.a1, joint.position.a2, joint.position.a3, joint.position.a4, joint.position.a5,
            joint.position.a6, joint.position.a7)

    def auto_calibrate_button_clicked(self):
        self.calibrate_thread.start()

    def manual_calibrate_button_clicked(self):
        tcp_pose = self.tcp_pose
        self.textEdit_calibrate.setText("开始手动标定,请尽量选取blabla位姿,以下显示ndi读数")
        with open('/home/lizq/win7share/NDI.txt', 'r') as ndi:
            ndi449 = ndi.read().splitlines()[0]
        if not "miss" in ndi449:
            self.manual_calibrate_count = 1
            with open('/home/lizq/win7share/ndi_data.txt', 'w') as f:  # 覆盖从头写入
                f.write(ndi449)
            self.textEdit_calibrate.append("point1:" + ndi449)
            with open('/home/lizq/win7share/robot_data.txt', 'w') as f:  # 覆盖从头写入
                f.write(','.join(str(i) for i in tcp_pose))
            # self.textEdit_calibrate.append("TCP:"+','.join(str(i) for i in tcp_pose))
        else:
            self.manual_calibrate_count = 0
            self.textEdit_calibrate.append("ndi数据包含miss,已自动移除")

    def add_calibrate_point_button_clicked(self):
        tcp_pose = self.tcp_pose
        with open('/home/lizq/win7share/NDI.txt', 'r') as ndi:
            ndi449 = ndi.read().splitlines()[0]
        if not "miss" in ndi449:
            self.manual_calibrate_count += 1
            # self.textEdit_calibrate.append("point "+str(self.manual_calibrate_count)+":")
            with open('/home/lizq/win7share/robot_data.txt', 'a') as f:  # 从末尾写入
                f.write("\n"+','.join(str(i) for i in tcp_pose))
            # self.textEdit_calibrate.append("TCP:" + ','.join(str(i) for i in tcp_pose))
            with open('/home/lizq/win7share/ndi_data.txt', 'a') as f:  # 从末尾写入
                f.write("\n"+ndi449)
            self.textEdit_calibrate.append("point "+str(self.manual_calibrate_count)+":" + ndi449)
        else:
            self.textEdit_calibrate.append("ndi数据包含miss,已自动移除")

    def delete_calibrate_point_button_clicked(self):
        self.manual_calibrate_count -= 1
        with open('/home/lizq/win7share/robot_data.txt', 'r') as f:
            lines = f.read().splitlines()
        with open('/home/lizq/win7share/robot_data.txt', 'w') as f:
            f.write('\n'.join(str(i) for i in lines[0:len(lines)-1]))
        with open('/home/lizq/win7share/ndi_data.txt', 'r') as f:
            lines = f.read().splitlines()
        with open('/home/lizq/win7share/ndi_data.txt', 'w') as f:
            f.write('\n'.join(str(i) for i in lines[0:len(lines)-1]))

    def finish_manual_calibrate_button_clicked(self):
        try:
            # 删除miss行
            # with open('/home/lizq/win7share/ndi_data.txt', 'r') as f:
            #     lines = f.readlines()
            # with open('/home/lizq/win7share/ndi_data.txt', 'w') as f:
            #     n = []
            #     i = 0
            #     for line in lines:
            #         if not "miss" in line:
            #             f.write(line)
            #             n.append(i)
            #         i += 1
            # with open('/home/lizq/win7share/robot_data.txt', 'r') as f:
            #     lines = f.readlines()
            # with open('/home/lizq/win7share/robot_data.txt', 'w') as f:
            #     for i in n:
            #         f.write(lines[i])
            self.matlab_eng.hand_eye_calibration(nargout=0)
            self.TJM = np.loadtxt('/home/lizq/win7share/TJM.txt', delimiter=",")  # mm
            self.textEdit_calibrate.setText(str(self.TJM))
            subprocess.call("cp /home/lizq/win7share/{TJM.txt,TOB.txt,TBN.txt} /home/lizq/win7share/手动标定矩阵保存", shell=True)
        except:
            self.textEdit_calibrate.setText("解算失败，请查看控制台说明")

    def calibrate_error_button_clicked(self):
        self.calibrate_error_thread.start()

    def ndi_error_button_clicked(self):
        tob = np.loadtxt('/home/lizq/win7share/TOB.txt', delimiter=",")  # mm
        tbo = np.linalg.inv(tob)
        ndi = np.genfromtxt('/home/lizq/win7share/NDI.txt', delimiter=",")  # 可以识别miss
        try:
            tjo1 = qc.quat2matrix(self.tcp_pose)  # m
            tjo1[0:3][:, 3] *= 1000  # mm
            p1 = qc.matrix2quat(tjo1)
        except:
            self.textEdit_calibrate.append("计算出错")

        tjm = self.TJM
        tmb = qc.quat2matrix(ndi[0].tolist())
        # print type(tjm),type(tmb)
        tjo2 = tjm.dot(tmb).dot(tbo)
        p2 = qc.matrix2quat(tjo2)
        distance,degree = qc.point_distance(p1,p2)
        self.textEdit_calibrate.setText(str(p1) + '\n' + str(p2))
        sentence = "位置偏差: %s mm" %distance
        self.textEdit_calibrate.append(sentence)
        sentence = "角度偏差: %s °" %degree
        self.textEdit_calibrate.append(sentence)

    def follow_button_clicked(self):
        self.follow_thread.start()

    def follow_pose_button_clicked(self):
        self.follow_pose_thread.start()

    def puncture_jinzhendian_button_clicked(self):
        self.puncture_point1 = np.genfromtxt('/home/lizq/win7share/NDI.txt', delimiter=",")[1].tolist()

    def puncture_chuancidian_button_clicked(self):
        self.puncture_point2 = np.genfromtxt('/home/lizq/win7share/NDI.txt', delimiter=",")[1].tolist()

    def puncture_cal_button_clicked(self):
        tjm = self.TJM  # mm
        ton = np.loadtxt('/home/lizq/win7share/TON.txt', delimiter=",")  # mm
        tgg = np.loadtxt('/home/lizq/win7share/TGG.txt', delimiter=",")  # mm
        tno = np.linalg.inv(ton)
        tmg1 = qc.quat2matrix(self.puncture_point1).dot(tgg)  # 矫正钢针针尖位置
        tmg2 = qc.quat2matrix(self.puncture_point2).dot(tgg)
        x = math.sqrt(
            (tmg2[0][3] - tmg1[0][3]) ** 2 + (tmg2[1][3] - tmg1[1][3]) ** 2 + (tmg2[2][3] - tmg1[2][3]) ** 2)  # 归一化
        xx = (tmg2[0][3] - tmg1[0][3]) / x
        xy = (tmg2[1][3] - tmg1[1][3]) / x
        xz = (tmg2[2][3] - tmg1[2][3]) / x  # TCP入针，即视觉空间的TCP（同穿刺针）的x方向

        y0x = -tmg1[0][0]
        y0y = -tmg1[1][0]
        y0z = -tmg1[2][0]  # y`方向为钢针-x方向

        zx, zy, zz = qc.chacheng(xx, xy, xz, y0x, y0y, y0z)  # x叉乘y`得到TCP在视觉空间的z方向，保证x的方向

        yx, yy, yz = qc.chacheng(zx, zy, zz, xx, xy, xz)  # z叉乘x得到y，使得刚体面向变化较小

        tmn_jinzhen = np.array([[xx, yx, zx, tmg1[0][3]],
                                [xy, yy, zy, tmg1[1][3]],
                                [xz, yz, zz, tmg1[2][3]],
                                [0.0, 0.0, 0.0, 1.0]])  # 进针点TMN位恣矩阵

        tmn_chuanci = np.array([[xx, yx, zx, tmg2[0][3]],
                                [xy, yy, zy, tmg2[1][3]],
                                [xz, yz, zz, tmg2[2][3]],
                                [0.0, 0.0, 0.0, 1.0]])  # 穿刺点TMN位恣矩阵
        tjo_1 = tjm.dot(tmn_jinzhen).dot(tno)
        tjo_1[0:3][:, 3] /= 1000
        self.TJO_jinzhen = tjo_1
        tjo_2 = tjm.dot(tmn_chuanci).dot(tno)
        tjo_2[0:3][:, 3] /= 1000
        self.TJO_chuanci = tjo_2

    def puncture_move_jinzhendian_button_clicked(self):
        self.pose_pub.publish(qc.get_command_pose(qc.matrix2quat(self.TJO_jinzhen)))

    def puncture_move_chuancidian_button_clicked(self):
        self.pose_pub.publish(qc.get_command_pose(qc.matrix2quat(self.TJO_chuanci)))

    def lineEdit_joint_out_copy_button_clicked(self):
        clipboard = QtWidgets.QApplication.clipboard()
        clipboard.setText(self.lineEdit_joint_out.text())

    def lineEdit_pose_out_copy_button_clicked(self):
        clipboard = QtWidgets.QApplication.clipboard()
        clipboard.setText(self.lineEdit_pose_out.text())

    def refresh_button_clicked(self):
        self.textEdit_calibrate.clear()
        self.textEdit_ct.clear()
        try:
            self.TJM = np.loadtxt('/home/lizq/win7share/TJM.txt', delimiter=",")  # mm
            # self.textEdit_calibrate.setText(str(self.TJM))
        except IOError:
            QtWidgets.QMessageBox.information(self, "提示", "未找到TJM标定矩阵，穿刺及跟随前请先执行标定程序")

    def switch_auto_matrix_button_clicked(self):
        subprocess.call("cp /home/lizq/win7share/自动标定矩阵保存/{TJM.txt,TOB.txt,TBN.txt} /home/lizq/win7share", shell=True)
        self.TJM = np.loadtxt('/home/lizq/win7share/TJM.txt', delimiter=",")  # mm
        self.textEdit_calibrate.append("已切换至自动标定所获得TJM,TOB,TBN矩阵")

    def switch_manual_matrix_button_clicked(self):
        subprocess.call("cp /home/lizq/win7share/手动标定矩阵保存/{TJM.txt,TOB.txt,TBN.txt} /home/lizq/win7share", shell=True)
        self.TJM = np.loadtxt('/home/lizq/win7share/TJM.txt', delimiter=",")  # mm
        self.textEdit_calibrate.append("已切换至手动标定所获得TJM,TOB,TBN矩阵")

    def server_button_clicked(self):
        self.server_thread.start()

    # region 输入姿态控制
    def joint_add_button_clicked(self):
        joint_now = list(self.joint_position)
        try:
            command_joint = list(eval(self.lineEdit_joint_in.text()))
            # print type(command_joint),type(joint_now),type(command_joint_add)
            if not len(command_joint) == 7:
                QtWidgets.QMessageBox.warning(self, "Warning", "输入位数错误")
                exit()  # sys.exit()会引发一个异常：SystemExit 被except捕捉
            for i in range(7):
                command_joint[i] = command_joint[i] / 180.0 * math.pi
                command_joint[i] = joint_now[i] + command_joint[i]
            command_line = qc.get_command_joint(command_joint)
        except:
            QtWidgets.QMessageBox.warning(self, "Warning", "输入错误")
            self.lineEdit_joint_in.clear()
        else:
            self.joint_pub.publish(command_line)
            rospy.loginfo(command_line)

    def joint_absolute_button_clicked(self):
        try:
            command_joint = list(eval(self.lineEdit_joint_in.text()))
            if not len(command_joint) == 7:
                QtWidgets.QMessageBox.warning(self, "Warning", "输入位数错误")
                exit()  # sys.exit()会引发一个异常：SystemExit 被except捕捉
            for i in range(7):
                command_joint[i] = command_joint[i] / 180.0 * math.pi
            command_line = qc.get_command_joint(command_joint)
        except:
            QtWidgets.QMessageBox.warning(self, "Warning", "输入错误")
            self.lineEdit_joint_in.clear()
        else:
            # print type(self.lineEdit_joint_in.text()),self.lineEdit_joint_in.text(),type(eval(self.lineEdit_joint_in.text())),eval(self.lineEdit_joint_in.text()),eval(self.lineEdit_joint_in.text())[0]
            self.joint_pub.publish(command_line)
            rospy.loginfo(command_line)

    def pose_add_button_clicked(self):
        tcp = list(self.tcp_pose)
        try:
            command_pose = list(eval(self.lineEdit_pose_in.text()))
            for i in range(0, 3):
                command_pose[i] /= 1000
            command_quat = qc.quat_pose_multipy(tcp, command_pose)
            command_line = qc.get_command_pose(command_quat)
        except:
            QtWidgets.QMessageBox.warning(self, "Warning", "输入错误")
            self.lineEdit_pose_in.clear()
        else:
            self.pose_pub.publish(command_line)
            rospy.loginfo(command_line)

    def pose_absolute_button_clicked(self):
        try:
            command_pose = list(eval(self.lineEdit_pose_in.text()))
            for i in range(0, 3):
                command_pose[i] /= 1000
            command_line = qc.get_command_pose(command_pose)
        except:
            QtWidgets.QMessageBox.warning(self, "Warning", "输入错误")
            self.lineEdit_pose_in.clear()
        else:
            self.pose_pub.publish(command_line)
            rospy.loginfo(command_line)

    # endregion

    # region 角度条按钮
    def joint_right_button_7_clicked(self):
        joint = list(self.joint_position)
        joint[6] += 0.01745329252  # 1度
        command_line = qc.get_command_joint(joint)
        self.joint_pub.publish(command_line)
        rospy.loginfo(command_line)

    def joint_right_button_6_clicked(self):
        joint = list(self.joint_position)
        joint[5] += 0.01745329252  # 1度
        command_line = qc.get_command_joint(joint)
        self.joint_pub.publish(command_line)
        rospy.loginfo(command_line)

    def joint_right_button_5_clicked(self):
        joint = list(self.joint_position)
        joint[4] += 0.01745329252  # 1度
        command_line = qc.get_command_joint(joint)
        self.joint_pub.publish(command_line)
        rospy.loginfo(command_line)

    def joint_right_button_4_clicked(self):
        joint = list(self.joint_position)
        joint[3] += 0.01745329252  # 1度
        command_line = qc.get_command_joint(joint)
        self.joint_pub.publish(command_line)
        rospy.loginfo(command_line)

    def joint_right_button_3_clicked(self):
        joint = list(self.joint_position)
        joint[2] += 0.01745329252  # 1度
        command_line = qc.get_command_joint(joint)
        self.joint_pub.publish(command_line)
        rospy.loginfo(command_line)

    def joint_right_button_2_clicked(self):
        joint = list(self.joint_position)
        joint[1] += 0.01745329252  # 1度
        command_line = qc.get_command_joint(joint)
        self.joint_pub.publish(command_line)
        rospy.loginfo(command_line)

    def joint_right_button_1_clicked(self):
        joint = list(self.joint_position)
        joint[0] += 0.01745329252  # 1度
        command_line = qc.get_command_joint(joint)
        self.joint_pub.publish(command_line)
        rospy.loginfo(command_line)

    def joint_left_button_7_clicked(self):
        joint = list(self.joint_position)
        joint[6] -= 0.01745329252  # 1度
        command_line = qc.get_command_joint(joint)
        self.joint_pub.publish(command_line)
        rospy.loginfo(command_line)

    def joint_left_button_6_clicked(self):
        joint = list(self.joint_position)
        joint[5] -= 0.01745329252  # 1度
        command_line = qc.get_command_joint(joint)
        self.joint_pub.publish(command_line)
        rospy.loginfo(command_line)

    def joint_left_button_5_clicked(self):
        joint = list(self.joint_position)
        joint[4] -= 0.01745329252  # 1度
        command_line = qc.get_command_joint(joint)
        self.joint_pub.publish(command_line)
        rospy.loginfo(command_line)

    def joint_left_button_4_clicked(self):
        joint = list(self.joint_position)
        joint[3] -= 0.01745329252  # 1度
        command_line = qc.get_command_joint(joint)
        self.joint_pub.publish(command_line)
        rospy.loginfo(command_line)

    def joint_left_button_3_clicked(self):
        joint = list(self.joint_position)
        joint[2] -= 0.01745329252  # 1度
        command_line = qc.get_command_joint(joint)
        self.joint_pub.publish(command_line)
        rospy.loginfo(command_line)

    def joint_left_button_2_clicked(self):
        joint = list(self.joint_position)
        joint[1] -= 0.01745329252  # 1度
        command_line = qc.get_command_joint(joint)
        self.joint_pub.publish(command_line)
        rospy.loginfo(command_line)

    def joint_left_button_1_clicked(self):
        joint = list(self.joint_position)
        joint[0] -= 0.01745329252  # 1度
        command_line = qc.get_command_joint(joint)
        self.joint_pub.publish(command_line)
        rospy.loginfo(command_line)

    # endregion

    # region 位姿控制按钮
    def position_z_add_button_clicked(self):
        tcp = list(self.tcp_pose)
        tcp[2] += 0.005  # 5毫米
        command_line = qc.get_command_pose(tcp)
        self.pose_pub.publish(command_line)
        rospy.loginfo(command_line)

    def position_z_min_button_clicked(self):
        tcp = list(self.tcp_pose)
        tcp[2] -= 0.005  # 5毫米
        command_line = qc.get_command_pose(tcp)
        self.pose_pub.publish(command_line)
        rospy.loginfo(command_line)

    def position_y_add_button_clicked(self):
        tcp = list(self.tcp_pose)
        tcp[1] += 0.005  # 5毫米
        command_line = qc.get_command_pose(tcp)
        self.pose_pub.publish(command_line)
        rospy.loginfo(command_line)

    def position_y_min_button_clicked(self):
        tcp = list(self.tcp_pose)
        tcp[1] -= 0.005  # 5毫米
        command_line = qc.get_command_pose(tcp)
        self.pose_pub.publish(command_line)
        rospy.loginfo(command_line)

    def position_x_add_button_clicked(self):
        tcp = list(self.tcp_pose)
        tcp[0] += 0.005  # 5毫米
        command_line = qc.get_command_pose(tcp)
        self.pose_pub.publish(command_line)
        rospy.loginfo(command_line)

    def position_x_min_button_clicked(self):
        tcp = list(self.tcp_pose)
        tcp[0] -= 0.005  # 5毫米
        command_line = qc.get_command_pose(tcp)
        self.pose_pub.publish(command_line)
        rospy.loginfo(command_line)

    def orientation_z_add_button_clicked(self):
        tcp = tuple(self.tcp_pose)
        add_tcp = (
            0.0, 0.0, 0.0, 0.0, 0.0, 0.008726535, 0.999961923)  # 0.008726535 = sin(0.5度) 0.999961923 = cos(0.5度) 即转1度
        command_tcp = qc.quat_pose_multipy(tcp, add_tcp)
        command_line = qc.get_command_pose(command_tcp)
        self.pose_pub.publish(command_line)
        rospy.loginfo(command_line)

    def orientation_z_min_button_clicked(self):
        tcp = tuple(self.tcp_pose)
        add_tcp = (
            0.0, 0.0, 0.0, 0.0, 0.0, -0.008726535, 0.999961923)  # 0.008726535 = sin(0.5度) 0.999961923 = cos(0.5度) 即转1度
        command_tcp = qc.quat_pose_multipy(tcp, add_tcp)
        command_line = qc.get_command_pose(command_tcp)
        self.pose_pub.publish(command_line)
        rospy.loginfo(command_line)

    def orientation_y_add_button_clicked(self):
        tcp = tuple(self.tcp_pose)
        add_tcp = (
            0.0, 0.0, 0.0, 0.0, 0.008726535, 0.0, 0.999961923)  # 0.008726535 = sin(0.5度) 0.999961923 = cos(0.5度) 即转1度
        command_tcp = qc.quat_pose_multipy(tcp, add_tcp)
        command_line = qc.get_command_pose(command_tcp)
        self.pose_pub.publish(command_line)
        rospy.loginfo(command_line)

    def orientation_y_min_button_clicked(self):
        tcp = tuple(self.tcp_pose)
        add_tcp = (
            0.0, 0.0, 0.0, 0.0, -0.008726535, 0.0, 0.999961923)  # 0.008726535 = sin(0.5度) 0.999961923 = cos(0.5度) 即转1度
        command_tcp = qc.quat_pose_multipy(tcp, add_tcp)
        command_line = qc.get_command_pose(command_tcp)
        self.pose_pub.publish(command_line)
        rospy.loginfo(command_line)

    def orientation_x_add_button_clicked(self):
        tcp = tuple(self.tcp_pose)
        add_tcp = (
            0.0, 0.0, 0.0, 0.008726535, 0.0, 0.0, 0.999961923)  # 0.008726535 = sin(0.5度) 0.999961923 = cos(0.5度) 即转1度
        command_tcp = qc.quat_pose_multipy(tcp, add_tcp)
        command_line = qc.get_command_pose(command_tcp)
        self.pose_pub.publish(command_line)
        rospy.loginfo(command_line)

    def orientation_x_min_button_clicked(self):
        tcp = tuple(self.tcp_pose)
        add_tcp = (
            0.0, 0.0, 0.0, -0.008726535, 0.0, 0.0, 0.999961923)  # 0.008726535 = sin(0.5度) 0.999961923 = cos(0.5度) 即转1度
        command_tcp = qc.quat_pose_multipy(tcp, add_tcp)
        command_line = qc.get_command_pose(command_tcp)
        self.pose_pub.publish(command_line)
        rospy.loginfo(command_line)

    # endregion

    # region 多线程槽函数们

    def textEdit_calibrate_append_slot(self,str):
        self.textEdit_calibrate.append(str)

    def textEdit_calibrate_settext_slot(self,str):
        self.textEdit_calibrate.setText(str)

    def textEdit_ct_settext_slot(self,str):
        self.textEdit_ct.setText(str)

    def textEdit_ct_append_slot(self,str):
        self.textEdit_ct.append(str)
    # endregion


class Calibrate_Thread(QtCore.QThread):
    append_signal = QtCore.pyqtSignal(str)
    # clear_signal = QtCore.pyqtSignal()
    settext_signal = QtCore.pyqtSignal(str)

    def __int__(self):
        super(Calibrate_Thread, self).__init__()

    def run(self):
        calibrate_start_pose = copy.deepcopy(window.tcp_pose)  # 深拷贝tuple,不受影响
        with open('/home/lizq/win7share/auto_calibrate_TCP.txt', 'w') as f:  # 记录TCP位置
            string = ','.join(str(i) for i in calibrate_start_pose)
            f.write(string)
        step = int(window.lineEdit_calibrate_step.text())
        lengh = float(window.lineEdit_calibrate_lengh.text())
        rad = float(window.lineEdit_calibrate_rad.text())
        # print step, type(step), lengh, type(lengh)
        try:
            for axs in ('rx', 'ry', 'rz', 'dx', 'dy', 'dz'):  # x,y,z旋转,x,y,z平移
                self.settext_signal[str].emit(axs)
                for n in range(1, step + 1):
                    calibrate_point = qc.turn_TCP_axs_rad_len(calibrate_start_pose, axs, rad * n, lengh * n)
                    # 为命令赋值
                    command_point = qc.get_command_pose(calibrate_point, n)
                    rospy.loginfo(command_point)
                    window.pose_pub.publish(command_point)
                    rate = rospy.Rate(0.7)  # 0.7hz
                    rate.sleep()
                    with open('/home/lizq/win7share/NDI.txt', 'r') as ndi:
                        ndi449 = ndi.read().splitlines()[0]
                        qc.write_to_txt(axs, n, ndi449)
                        self.append_signal[str].emit(ndi449)

                    # 手动标定算法测试
                    with open('/home/lizq/win7share/robot_data.txt', 'a') as f:  # 从末尾写入
                        f.write("\n" + ','.join(str(i) for i in window.tcp_pose))
                    with open('/home/lizq/win7share/NDI.txt', 'r') as ndi:
                        ndi449 = ndi.read().splitlines()[0]
                    with open('/home/lizq/win7share/ndi_data.txt', 'a') as f:  # 从末尾写入
                        f.write("\n" + ndi449)

                    if not window.auto_calibrate_button.isChecked():
                        exit()  # sys.exit()会引发一个异常：SystemExit 被except捕捉
                command_point = qc.get_command_pose(calibrate_start_pose)
                rospy.loginfo(command_point)
                window.pose_pub.publish(command_point)
                rate = rospy.Rate(0.3)  # 0.3hz
                rate.sleep()
                if not window.auto_calibrate_button.isChecked():
                    exit()  # sys.exit()会引发一个异常：SystemExit 被except捕捉
            window.matlab_eng.MinTwoSolveTJM(nargout=0)
        except SystemExit:
            self.append_signal[str].emit("!!标定已人为中止!!")
        except:
            self.settext_signal[str].emit("!!标定失败,请查看控制台说明!!")
        else:
            window.TJM = np.loadtxt('/home/lizq/win7share/TJM.txt', delimiter=",")  # mm 更新TJM
            self.settext_signal[str].emit(str(window.TJM))
            subprocess.call("cp /home/lizq/win7share/{TJM.txt,TOB.txt,TBN.txt} /home/lizq/win7share/自动标定矩阵保存", shell=True)
        window.auto_calibrate_button.setChecked(False)


class Calibrate_Error_Thread(QtCore.QThread):
    append_signal = QtCore.pyqtSignal(str)
    settext_signal = QtCore.pyqtSignal(str)

    def __int__(self):
        super(Calibrate_Error_Thread, self).__init__()

    def run(self):
        tbo = np.linalg.inv(np.loadtxt('/home/lizq/win7share/TOB.txt', delimiter=","))  # mm
        p1 = np.genfromtxt('/home/lizq/win7share/NDI.txt', delimiter=",")[0]
        with open('/home/lizq/win7share/NDI.txt', 'r') as ndi:
            p1_output = ndi.read().splitlines()[0]
        self.settext_signal[str].emit("出发点位姿："+ p1_output)
        tmb = qc.quat2matrix(p1)
        try:
            joint = list(eval(window.textEdit_calibrate.toPlainText()))
            for i in range(7):
                joint[i] *= math.pi/180.0
            backpoint = qc.get_command_joint(joint)
        except:
            self.append_signal[str].emit("请在标定对话框中输入附近的角关节空间坐标,如\n-50, 30, 0, -90, 0, -90, 0")
        else:
            window.joint_pub.publish(backpoint)
            rospy.sleep(3)

            tjo = window.TJM.dot(tmb).dot(tbo)  # 可能误差出现在tjm,tbo
            tjo[0:3][:, 3] /= 1000  # mm->m
            command_point = qc.matrix2quat(tjo)
            command_line = qc.get_command_pose(command_point)
            window.pose_pub.publish(command_line)
            rospy.sleep(3)

            p2 = np.genfromtxt('/home/lizq/win7share/NDI.txt', delimiter=",")[0]
            with open('/home/lizq/win7share/NDI.txt', 'r') as ndi:
                p2_output = ndi.read().splitlines()[0]
            self.append_signal[str].emit("返回点位姿：" + p2_output)
            distance, degree = qc.point_distance(p1, p2)
            sentence = "位置偏差: %s mm\n角度偏差: %s °" %(distance,degree)
            self.append_signal[str].emit(sentence)


class Follow_Thread(QtCore.QThread):
    settext_signal = QtCore.pyqtSignal(str)

    def __int__(self):
        super(Follow_Thread, self).__init__()

    def run(self):
        rate = rospy.Rate(50)  # smart servo 可以达到 20ms 50hz
        ton = np.loadtxt('/home/lizq/win7share/TON.txt', delimiter=",")  # mm 需要优化
        tno = np.linalg.inv(ton)
        tcp = list(window.tcp_pose)  # 米 只取姿态
        tgg = np.loadtxt('/home/lizq/win7share/TGG.txt', delimiter=",")  # mm 需要有话
        tbn = np.loadtxt('/home/lizq/win7share/TBN.txt', delimiter=",")
        while window.follow_button.isChecked():  # 等待NDI数据
            try:
                ndi = np.genfromtxt('/home/lizq/win7share/NDI.txt', delimiter=",")
                if math.isnan(ndi[1][0]):
                    self.settext_signal[str].emit("等待钢针")
                else:
                    tmg = qc.quat2matrix(ndi[1].tolist())  # 被动刚体位姿
                    tmg = tmg.dot(tgg)  # 更正钢针位姿
                    tjg = window.TJM.dot(tmg)  # 将钢针针尖位置变换至基座坐标系下

                    # 发射针尖与钢针间距离
                    if not math.isnan(ndi[0][0]):  # 可以看到449时，比较ndi下tgg矫正的tmg和tbn矫正的tmn，误差在两个矫正矩阵
                        tmb = qc.quat2matrix(ndi[0].tolist())
                        tmn = tmb.dot(tbn)
                        distance = ((tmn[0][3] - tmg[0][3])**2 + (tmg[1][3] - tmn[1][3])**2 +(tmg[2][3] - tmn[2][3])**2)**0.5
                        sentence = "ndi下钢针位置：\n%s\nndi下穿刺针位置\n%s\n针尖距离：%fmm" % (tmg, tmn, distance)
                    else:  # 看不到449时，针尖位姿通过关节角度传感器与ton取得，误差来源多了TJM,还算得慢
                        tjo = qc.quat2matrix(tcp)
                        tjo[0:3][:, 3] *= 1000
                        tjn = tjo.dot(ton)
                        distance = ((tjg[0][3] - tjn[0][3])**2 + (tjg[1][3] - tjn[1][3])**2 + (tjg[2][3] - tjn[2][3])**2)**0.5
                        sentence = "基座下下钢针位置：\n%s\n基座下穿刺针位置\n%s\n针尖距离：%fmm" % (tjg,tjn,distance)
                    self.settext_signal[str].emit(sentence)

                    gangzhen = list(qc.matrix2quat(tjg))
                    gangzhen[3:] = tcp[3:]  # 使得TCP姿态不变，被动刚体朝向NDI，只做位移
                    tjn = qc.quat2matrix(gangzhen)
                    tjo = tjn.dot(tno)
                    tjo[0:3][:, 3] /= 1000  # mm->m
                    command_point = qc.get_command_pose(qc.matrix2quat(tjo))
                    rospy.loginfo(command_point)
                    window.pose_pub.publish(command_point)
                    rate.sleep()
            except:
                pass


class Follow_Pose_Thread(QtCore.QThread):
    def __int__(self):
        super(Follow_Pose_Thread, self).__init__()

    def run(self):
        tjm = window.TJM
        rate = rospy.Rate(50)  # smart servo 可以达到 20ms 50hz
        ton = np.loadtxt('/home/lizq/win7share/TON.txt', delimiter=",")  # mm 需要优化
        tno = np.linalg.inv(ton)
        tgg = np.loadtxt('/home/lizq/win7share/TGG.txt', delimiter=",")  # mm 需要有话
        while window.follow_pose_button.isChecked():  # 等待NDI数据
            try:
                ndi = np.genfromtxt('/home/lizq/win7share/NDI.txt', delimiter=",")
                if math.isnan(ndi[1][0]):
                    print "waiting"
                else:
                    tmg = qc.quat2matrix(ndi[1].tolist())  # 被动刚体位姿
                    tmg = tmg.dot(tgg)  # 更正钢针针尖位姿
                    tjg = tjm.dot(tmg)  # 将钢针针尖位置变换至基座坐标系下
                    # 穿刺针x为钢针z方向，穿刺针y方向为钢针-x,因此穿刺针z方向为钢针-y
                    tjn = np.array([[tjg[0][2], -tjg[0][0], -tjg[0][1], tjg[0][3]],
                                    [tjg[1][2], -tjg[1][0], -tjg[1][1], tjg[1][3]],
                                    [tjg[2][2], -tjg[2][0], -tjg[2][1], tjg[2][3]],
                                    [0.0, 0.0, 0.0, 1.0]])
                    tjo = tjn.dot(tno)
                    tjo[0:3][:, 3] /= 1000  # mm->m
                    command_point = qc.get_command_pose(qc.matrix2quat(tjo))
                    rospy.loginfo(command_point)
                    window.pose_pub.publish(command_point)
                    rate.sleep()
            except:
                pass


class Server_Thread(QtCore.QThread):
    settext_signal = QtCore.pyqtSignal(str)
    append_signal = QtCore.pyqtSignal(str)
    def __int__(self):
        super(Server_Thread, self).__init__()

    def run(self):
        ip = window.lineEdit_ip.text()
        port = int(window.lineEdit_port.text())
        sk = socket.socket()  # 生成一个句柄
        try:  #
            sk.bind((ip,port))  # 绑定ip端口
            sk.listen(5)  # 最多连接数
        except Exception as e:
            print e
            self.append_signal[str].emit("启动失败")
            window.server_button.setChecked(False)
        else:
            self.settext_signal[str].emit("等待CT端连接")
            conn, addr = sk.accept()  # 等待链接,阻塞，直到渠道链接 conn打开一个新的对象 专门给当前链接的客户端 addr是ip地址
            self.append_signal[str].emit("已连接")
            while window.server_button.isChecked():
                try:
                    # 获取客户端请求数据
                    client_data = conn.recv(1024)  # 接受套接字的数据。数据以字符串形式返回，bufsize指定最多可以接收的数量。
                    self.append_signal[str].emit(str(client_data))  # 打印对方的数据
                    reply = self.check_out(client_data)  # 处理数据
                    conn.sendall(reply)  # 向对方发送数据
                # 客户端发送flag，比如0就返回针尖位置，可以循环发0
                # 再写个客户端的界面，可以手写
                except Exception as e:
                    print e
                    traceback.print_exc()
                    window.server_button.setChecked(False)
                    self.append_signal[str].emit("连接已断开")
            # 关闭链接
            conn.close()

    def check_out(self,client_data):
        if "ask for tmn" in client_data:
            # ndi数据
            try:
                ndi449 = np.genfromtxt('/home/lizq/win7share/NDI.txt', delimiter=",")[0]
                tbn = np.loadtxt('/home/lizq/win7share/TBN.txt', delimiter=",")
                tmn = qc.quat2matrix(ndi449).dot(tbn)
            except:
                reply = 'tmn is lost in ndi'
            else:
                reply = str(qc.matrix2quat(tmn))
            # 机械臂数据
            # try:
            #     tjm = np.loadtxt('/home/lizq/win7share/TJM.txt', delimiter=",")
            #     ton = np.loadtxt('/home/lizq/win7share/TON.txt', delimiter=",")
            #     tjo = qc.quat2matrix(window.tcp_pose)
            #     tjo[0:3][:, 3] *= 1000
            #     tmn = np.linalg.inv(tjm)*tjo*ton
            # except:
            #     reply = '解算失败'
            # else:
            #     reply = qc.matrix2quat(tmn)

        elif "send tmc quat" in client_data:
            data = client_data.splitlines()[1]
            quat = eval(data)
            fenmu = math.sqrt(quat[3] **2 + quat[4] **2 +quat[5] **2 +quat[6] **2)
            x, y, z, w = [i / fenmu for i in [quat[3], quat[4], quat[5], quat[6]]]
            self.tmc_quat = (quat[0],quat[1],quat[2],x,y,z,w)
            reply = "tmc received"

        elif "move to point" in client_data:
            data = client_data.splitlines()[1]
            quat = eval(data)
            try:
                tmp = qc.quat_pose_multipy(self.tmc_quat,quat)
                tjp = window.TJM.dot(qc.quat2matrix(tmp))
                tjp[0:3][:, 3] /= 1000
                window.pose_pub.publish(qc.get_command_pose(qc.matrix2quat(tjp)))
                rospy.loginfo(qc.get_command_pose(qc.matrix2quat(tjp)))
            except Exception as e:
                print e
                reply = "please send tmc first"
            else:
                reply = "command received"

        elif "send entry point" in client_data:
            data = client_data.splitlines()[1]
            self.tcn1 = eval(data)
            reply = "command received"

        elif "send puncture point" in client_data:
            data = client_data.splitlines()[1]
            self.tcn2 = eval(data)
            reply = "command received"

        elif "move to entry point" in client_data:
            tjm = window.TJM  # mm
            ton = np.loadtxt('/home/lizq/win7share/TON.txt', delimiter=",")  # mm
            tno = np.linalg.inv(ton)
            try:
                tmn1 = qc.quat_pose_multipy(self.tmc_quat,self.tcn1)
                tmn2 = qc.quat_pose_multipy(self.tmc_quat,self.tcn2)
                # tmn1 = qc.quat2matrix(tmn1)
                # tmn2 = qc.quat2matrix(tmn2)
            except Exception as e:
                print e
                reply = "please send tmc and puncture path fisrt"
            else:

                # TCP入针，即视觉空间的TCP（同穿刺针）的x方向
                # fenmu = math.sqrt((tmn2[0][3] - tmn1[0][3]) ** 2 + (tmn2[1][3] - tmn1[1][3]) ** 2 + (tmn2[2][3] - tmn1[2][3]) ** 2)  # 归一化
                # xx = (tmn2[0][3] - tmn1[0][3]) / fenmu
                # xy = (tmn2[1][3] - tmn1[1][3]) / fenmu
                # xz = (tmn2[2][3] - tmn1[2][3]) / fenmu
                # 还是用原来光学探针采的姿态
                # y0x = -tmn1[0][0]
                # y0y = -tmn1[1][0]
                # y0z = -tmn1[2][0]  # 穿刺针y方向为钢针-x方向

                # 若CT端只发位置过来 即tcn四元数为0,0,0,1
                fenmu = math.sqrt((tmn2[0] - tmn1[0]) ** 2 + (tmn2[1] - tmn1[1]) ** 2 + (tmn2[2] - tmn1[2]) ** 2)  # 归一化
                xx = (tmn2[0] - tmn1[0]) / fenmu
                xy = (tmn2[1] - tmn1[1]) / fenmu
                xz = (tmn2[2] - tmn1[2]) / fenmu
                tcp = qc.quat2matrix(window.tcp_pose)
                y0x = tcp[0][1]
                y0y = tcp[1][1]
                y0z = tcp[2][1]  # 穿刺针y方向应取当前tcp y方向附近，可以被ndi所视方向

                zx, zy, zz = qc.chacheng(xx, xy, xz, y0x, y0y, y0z)  # x叉乘y`得到TCP在视觉空间的z方向，保证x的方向
                yx, yy, yz = qc.chacheng(zx, zy, zz, xx, xy, xz)  # z叉乘x得到y，使得刚体面向变化较小
                tmn_jinzhen = np.array([[xx, yx, zx, tmn1[0]],
                                        [xy, yy, zy, tmn1[1]],
                                        [xz, yz, zz, tmn1[2]],
                                        [0.0, 0.0, 0.0, 1.0]])  # 进针点TMN位恣矩阵
                tmn_chuanci = np.array([[xx, yx, zx, tmn2[0]],
                                        [xy, yy, zy, tmn2[1]],
                                        [xz, yz, zz, tmn2[2]],
                                        [0.0, 0.0, 0.0, 1.0]])  # 穿刺点TMN位恣矩阵
                tjo_1 = tjm.dot(tmn_jinzhen).dot(tno)
                tjo_1[0:3][:, 3] /= 1000
                self.TJO_jinzhen = tjo_1
                tjo_2 = tjm.dot(tmn_chuanci).dot(tno)
                tjo_2[0:3][:, 3] /= 1000
                self.TJO_chuanci = tjo_2

                window.pose_pub.publish(qc.get_command_pose(qc.matrix2quat(self.TJO_jinzhen)))
                rospy.loginfo(qc.get_command_pose(qc.matrix2quat(self.TJO_jinzhen)))
                reply = "command received"

        elif "move to puncture point" in client_data:
            window.pose_pub.publish(qc.get_command_pose(qc.matrix2quat(self.TJO_chuanci)))
            rospy.loginfo(qc.get_command_pose(qc.matrix2quat(self.TJO_chuanci)))
            reply = "command received"

        else:
            print client_data
            reply = "wrong command"

        return reply



app = QtWidgets.QApplication(sys.argv)
window = Mywindow()
window.show()
sys.exit(app.exec_())

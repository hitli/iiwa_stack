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
import matlab.engine
import img_rcc_rc


class Mywindow(QtWidgets.QMainWindow, Ui_MainWindow):
    def __init__(self):

        # 全局变量
        self.joint_position = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)  # a1,a2,a3,a4,a5,a6,a7 tuple 弧度
        self.tcp_pose = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)  # x,y,z,rx,ry,rz,rw tuple 米
        self.TJM = np.loadtxt('/home/lizq/win7share/TJM.txt', delimiter=",")  # mm
        self.manual_calibrate_count = 1

        super(Mywindow, self).__init__()
        self.setupUi(self)
        self.ndi_error_button.setToolTip("通过角关节位置与TOB计算TJB(基本无误差),与TJM*TMB=TJB比较,查看TJM误差")
        self.calibrate_error_button.setToolTip("记录449位置,命令末端运动到附近再返回记录位置,比较ndi数据距离,确定重定位精度")
        self.manual_calibrate_button.setToolTip("初始化ndi,robot_data文件,需要可以看到449")
        #  启动matlab核心
        # self.matlab_eng = matlab.engine.start_matlab()

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
        # self.calibrate_thread.clear_signal.connect(self.textEdit_calibrate_clear_slot)
        self.calibrate_thread.settext_signal[str].connect(self.textEdit_calibrate_settext_slot)

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

        with open('/home/lizq/win7share/NDI.txt', 'r') as ndi:
            ndi = ndi.read().splitlines()
            self.lineEdit_ndi449.setText(ndi[0])
            self.lineEdit_ndi340.setText(ndi[1])
            self.lineEdit_ndi339.setText(ndi[2])

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
        self.manual_calibrate_count = 1
        self.textEdit_calibrate.setText("开始手动标定,请尽量选取blabla位姿"+'\n'+"point 1:")
        with open('/home/lizq/win7share/robot_data.txt', 'w') as f:  # 覆盖从头写入
            f.write(str(self.tcp_pose))
        self.textEdit_calibrate.append("TCP:"+str(self.tcp_pose))
        with open('/home/lizq/win7share/NDI.txt', 'r') as ndi:
            ndi449 = ndi.read().splitlines()[0]
        with open('/home/lizq/win7share/ndi_data.txt', 'w') as f:  # 覆盖从头写入
            f.write(ndi449)
        self.textEdit_calibrate.append("ndi:"+ndi449)

    def add_calibrate_point_button_clicked(self):
        self.manual_calibrate_count += 1
        self.textEdit_calibrate.append("point "+str(self.manual_calibrate_count)+":")
        with open('/home/lizq/win7share/robot_data.txt', 'a') as f:  # 从末尾写入
            f.write(str(self.tcp_pose))
        self.textEdit_calibrate.append("TCP:" + str(self.tcp_pose))
        with open('/home/lizq/win7share/NDI.txt', 'r') as ndi:
            ndi449 = ndi.read().splitlines()[0]
        with open('/home/lizq/win7share/ndi_data.txt', 'a') as f:  # 从末尾写入
            f.write(ndi449)
        self.textEdit_calibrate.append("ndi:" + ndi449)

    def finish_manual_calibrate_button_clicked(self):
        self.matlab_eng.hand_eye_calibration(nargout=0)
        self.TJM = np.loadtxt('/home/lizq/win7share/TJM.txt', delimiter=",")  # mm
        self.textEdit_calibrate.setText(self.TJM)


    def calibrate_error_button_clicked(self):
        tbo = np.loadtxt('/home/lizq/win7share/TBO.txt', delimiter=",")  # mm
        p1 = np.genfromtxt('/home/lizq/win7share/NDI.txt', delimiter=",")[0]
        tmb = qc.quat2matrix(p1)
        backpoint = qc.get_command_joint((-50, 30, 0, -90, 0, -90, 0))
        self.joint_pub.publish(backpoint)
        rospy.sleep(3)

        tjo = self.TJM.dot(tmb).dot(tbo)  # 可能误差出现在tjm,tbo
        tjo[0:3][:, 3] /= 1000  # mm->m
        command_point = qc.matrix2quat(tjo)
        command_line = qc.get_command_pose(command_point)
        self.pose_pub.publish(command_line)
        rospy.sleep(3)

        p2 = np.genfromtxt('/home/lizq/win7share/NDI.txt', delimiter=",")[0]
        distance, degree = qc.point_distance(p1, p2)
        self.textEdit_calibrate.setText(str(p1) + '\n' + str(p2))
        sentence = "位置偏差:" + str(distance) + "mm"
        self.textEdit_calibrate.append(sentence)
        sentence = "角度偏差:" + str(degree) + "°"
        self.textEdit_calibrate.append(sentence)

    def ndi_error_button_clicked(self):
        tob = np.loadtxt('/home/lizq/win7share/TOB.txt', delimiter=",")  # mm
        tjo = qc.quat2matrix(self.tcp_pose)  # m
        tjo[0:3][:, 3] *= 1000  # mm
        tjb1 = tjo.dot(tob)
        p1 = qc.matrix2quat(tjb1)
        tjm = self.TJM
        with open('/home/lizq/win7share/NDI.txt', 'r') as ndi:
            tmb = ndi.read().splitlines()[0]
        tjb2 = tjm.dot(tmb)
        p2 = qc.matrix2quat(tjb2)
        distance,degree = qc.point_distance(p1,p2)
        self.textEdit_calibrate.setText(str(p1) + '\n' + str(p2))
        sentence = "位置偏差:" + str(distance) + "mm"
        self.textEdit_calibrate.append(sentence)
        sentence = "角度偏差:" + str(degree) + "°"
        self.textEdit_calibrate.append(sentence)


    def follow_button_clicked(self):
        self.follow_thread.start()

    def follow_pose_button_clicked(self):
        pass

    def puncture_test_button_clicked(self):
        pass

    def lineEdit_joint_out_copy_button_clicked(self):
        clipboard = QtWidgets.QApplication.clipboard()
        clipboard.setText(self.lineEdit_joint_out.text())

    def lineEdit_pose_out_copy_button_clicked(self):
        clipboard = QtWidgets.QApplication.clipboard()
        clipboard.setText(self.lineEdit_pose_out.text())

    def refresh_button_clicked(self):
        self.TJM = np.loadtxt('/home/lizq/win7share/TJM.txt', delimiter=",")  # mm

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
            command_quat = qc.quat_matrix_multipy(tcp, command_pose)
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
        tcp[2] += 0.001  # 1毫米
        command_line = qc.get_command_pose(tcp)
        self.pose_pub.publish(command_line)
        rospy.loginfo(command_line)

    def position_z_min_button_clicked(self):
        tcp = list(self.tcp_pose)
        tcp[2] -= 0.001  # 1毫米
        command_line = qc.get_command_pose(tcp)
        self.pose_pub.publish(command_line)
        rospy.loginfo(command_line)

    def position_y_add_button_clicked(self):
        tcp = list(self.tcp_pose)
        tcp[1] += 0.001  # 1毫米
        command_line = qc.get_command_pose(tcp)
        self.pose_pub.publish(command_line)
        rospy.loginfo(command_line)

    def position_y_min_button_clicked(self):
        tcp = list(self.tcp_pose)
        tcp[1] -= 0.001  # 1毫米
        command_line = qc.get_command_pose(tcp)
        self.pose_pub.publish(command_line)
        rospy.loginfo(command_line)

    def position_x_add_button_clicked(self):
        tcp = list(self.tcp_pose)
        tcp[0] += 0.001  # 1毫米
        command_line = qc.get_command_pose(tcp)
        self.pose_pub.publish(command_line)
        rospy.loginfo(command_line)

    def position_x_min_button_clicked(self):
        tcp = list(self.tcp_pose)
        tcp[0] -= 0.001  # 1毫米
        command_line = qc.get_command_pose(tcp)
        self.pose_pub.publish(command_line)
        rospy.loginfo(command_line)

    def orientation_z_add_button_clicked(self):
        tcp = tuple(self.tcp_pose)
        add_tcp = (
            0.0, 0.0, 0.0, 0.0, 0.0, 0.008726535, 0.999961923)  # 0.008726535 = sin(0.5度) 0.999961923 = cos(0.5度) 即转1度
        command_tcp = qc.quat_matrix_multipy(tcp, add_tcp)
        command_line = qc.get_command_pose(command_tcp)
        self.pose_pub.publish(command_line)
        rospy.loginfo(command_line)

    def orientation_z_min_button_clicked(self):
        tcp = tuple(self.tcp_pose)
        add_tcp = (
            0.0, 0.0, 0.0, 0.0, 0.0, -0.008726535, 0.999961923)  # 0.008726535 = sin(0.5度) 0.999961923 = cos(0.5度) 即转1度
        command_tcp = qc.quat_matrix_multipy(tcp, add_tcp)
        command_line = qc.get_command_pose(command_tcp)
        self.pose_pub.publish(command_line)
        rospy.loginfo(command_line)

    def orientation_y_add_button_clicked(self):
        tcp = tuple(self.tcp_pose)
        add_tcp = (
            0.0, 0.0, 0.0, 0.0, 0.008726535, 0.0, 0.999961923)  # 0.008726535 = sin(0.5度) 0.999961923 = cos(0.5度) 即转1度
        command_tcp = qc.quat_matrix_multipy(tcp, add_tcp)
        command_line = qc.get_command_pose(command_tcp)
        self.pose_pub.publish(command_line)
        rospy.loginfo(command_line)

    def orientation_y_min_button_clicked(self):
        tcp = tuple(self.tcp_pose)
        add_tcp = (
            0.0, 0.0, 0.0, 0.0, -0.008726535, 0.0, 0.999961923)  # 0.008726535 = sin(0.5度) 0.999961923 = cos(0.5度) 即转1度
        command_tcp = qc.quat_matrix_multipy(tcp, add_tcp)
        command_line = qc.get_command_pose(command_tcp)
        self.pose_pub.publish(command_line)
        rospy.loginfo(command_line)

    def orientation_x_add_button_clicked(self):
        tcp = tuple(self.tcp_pose)
        add_tcp = (
            0.0, 0.0, 0.0, 0.008726535, 0.0, 0.0, 0.999961923)  # 0.008726535 = sin(0.5度) 0.999961923 = cos(0.5度) 即转1度
        command_tcp = qc.quat_matrix_multipy(tcp, add_tcp)
        command_line = qc.get_command_pose(command_tcp)
        self.pose_pub.publish(command_line)
        rospy.loginfo(command_line)

    def orientation_x_min_button_clicked(self):
        tcp = tuple(self.tcp_pose)
        add_tcp = (
            0.0, 0.0, 0.0, -0.008726535, 0.0, 0.0, 0.999961923)  # 0.008726535 = sin(0.5度) 0.999961923 = cos(0.5度) 即转1度
        command_tcp = qc.quat_matrix_multipy(tcp, add_tcp)
        command_line = qc.get_command_pose(command_tcp)
        self.pose_pub.publish(command_line)
        rospy.loginfo(command_line)

    # endregion

    # region 多线程槽函数们

    def textEdit_calibrate_append_slot(self,str):
        self.textEdit_calibrate.append(str)

    def textEdit_calibrate_clear_slot(self):
        self.textEdit_calibrate.clear()

    def textEdit_calibrate_settext_slot(self,str):
        self.textEdit_calibrate.setText(str)

    # endregion


class Calibrate_Thread(QtCore.QThread):
    append_signal = QtCore.pyqtSignal(str)
    # clear_signal = QtCore.pyqtSignal()
    settext_signal = QtCore.pyqtSignal(str)

    def __int__(self):
        super(Calibrate_Thread, self).__init__()

    def run(self):
        calibrate_start_pose = copy.deepcopy(window.tcp_pose)  # 深拷贝tuple,不受影响
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
            self.append_signal[str].emit("!!标定失败,请查看控制台说明!!")
        else:
            window.TJM = np.loadtxt('/home/lizq/win7share/TJM.txt', delimiter=",")  # mm 更新TJM
            self.settext_signal[str].emit(str(window.TJM))

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
        while window.follow_button.isChecked():  # 等待NDI数据
            try:
                ndi = np.genfromtxt('/home/lizq/win7share/NDI.txt', delimiter=",")
                if math.isnan(ndi[1][0]):
                    self.settext_signal[str].emit("等待钢针")
                else:
                    tmg = qc.quat2matrix(ndi[1].tolist())  # 被动刚体位姿
                    tmg = tmg.dot(tgg)  # 更正钢针位姿
                    tjg = window.TJM.dot(tmg)  # 将钢针针尖位置变换至基座坐标系下

                    # 发射针尖目标距离,针尖位姿通过关节角度传感器与ton取得
                    # 还可以通过ndi看449与tbn获得,跳过了tjm误差,但是tbn没算
                    tjo = qc.quat2matrix(tcp)
                    tjo[0:3][:, 3] *= 1000
                    tjn = tjo.dot(ton)
                    # print tjn
                    # print tjg
                    distance = pow(
                        pow(tjg[0][3] - tjn[0][3], 2) + pow(tjg[1][3] - tjn[1][3], 2) + pow(tjg[2][3] - tjn[2][3], 2),
                        0.5)
                    self.settext_signal[str].emit(str(distance))

                    gangzhen = list(qc.matrix2quat(tjg))
                    gangzhen[3:] = tcp[3:]  # 使得TCP姿态不变，被动刚体朝向NDI，只做位移
                    tjn = qc.quat2matrix(gangzhen)
                    tjo = tjn.dot(tno)
                    tjo[0:3][:, 3] /= 1000  # mm->m
                    command_point = qc.get_command_pose(qc.matrix2quat(tjo))
                    rospy.loginfo(command_point)
                    window.pub.publish(command_point)
                    rate.sleep()
            except:
                pass


app = QtWidgets.QApplication(sys.argv)
window = Mywindow()
window.show()
sys.exit(app.exec_())

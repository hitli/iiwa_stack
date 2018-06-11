# -*- coding: utf-8 -*-

from mainwindow import Ui_MainWindow
from PyQt5 import QtWidgets,QtCore,QtGui
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
        self.joint_position = (0.0,0.0,0.0,0.0,0.0,0.0,0.0)  # a1,a2,a3,a4,a5,a6,a7 tuple 角度
        self.tcp_pose = (0.0,0.0,0.0,0.0,0.0,0.0,0.0)  # x,y,z,rx,ry,rz,rw tuple 米
        self.TJM = np.loadtxt('/home/lizq/win7share/TJM.txt', delimiter=",")  # mm

        super(Mywindow, self).__init__()
        self.setupUi(self)

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
        self.follow_thread.waiting_signal.connect(self.follow_waiting_slot)  # 线程.信号.connect(槽)
        self.follow_thread.distance_signal[float].connect(self.follow_distance_slot)

    def read_state(self):

        # ','.join(str(i) for i in self.tcp_pose)将数组每一位以,间隔转为字符串
        tcp_pose_output = list(self.tcp_pose)
        for i in range(3):
            tcp_pose_output[i] = round(tcp_pose_output[i]*1000,3)
        for i in range(3,7):
            tcp_pose_output[i] = round(tcp_pose_output[i], 3)
        self.lineEdit_pose_out.setText(','.join(str(i) for i in tcp_pose_output))

        joint_position_output= list(self.joint_position)  # 数组化方便操作,tuple不让运算
        for i in range(7):
            joint_position_output[i] = round(joint_position_output[i]*180.0/math.pi,2)
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
        self.tcp_pose = (tcp.pose.position.x, tcp.pose.position.y, tcp.pose.position.z, tcp.pose.orientation.x,tcp.pose.orientation.y, tcp.pose.orientation.z, tcp.pose.orientation.w)

    def get_joint_position(self, joint):
        self.joint_position = (joint.position.a1, joint.position.a2, joint.position.a3, joint.position.a4, joint.position.a5, joint.position.a6, joint.position.a7)

    def joint_add_button_clicked(self):
        pass

    def joint_absolute_button_clicked(self):
        command_joint = eval(self.lineEdit_joint_in.text())
        command_line = qc.get_command_joint(command_joint)
        self.joint_pub.publish(command_line)
        rospy.loginfo(command_line)

    def pose_add_button_clicked(self):
        pass

    def pose_absolute_button_clicked(self):
        pass

    def auto_calibrate_button_clicked(self):

        calibrate_start_pose = copy.deepcopy(self.tcp_pose)  # 深拷贝tuple,不受影响
        step = self.lineEdit_calibrate_step.text()
        lengh = self.lineEdit_calibrate_lengh.text()
        rad = self.lineEdit_calibrate_rad.text()

        for axs in ('rx', 'ry', 'rz', 'dx', 'dy', 'dz'):  # x,y,z旋转,x,y,z平移
            for n in range(1, step + 1):
                calibrate_point = qc.turn_TCP_axs_rad_len(calibrate_start_pose, axs, rad * n, lengh * n)
                # 为命令赋值
                command_point = qc.get_command_pose(calibrate_point, n)
                rospy.loginfo(command_point)
                self.pose_pub.publish(command_point)
                rate = rospy.Rate(0.7)  # 0.7hz
                rate.sleep()
                with open('/home/lizq/win7share/NDI.txt', 'r') as ndi:
                    qc.write_to_txt(axs, n, ndi.read().splitlines()[0])
            command_point = qc.get_command_pose(calibrate_start_pose)
            rospy.loginfo(command_point)
            self.pose_pub.publish(command_point)
            rate = rospy.Rate(0.3)  # 0.3hz
            rate.sleep()
        self.matlab_eng.MinTwoSolveTJM(nargout=0)
        self.TJM = np.loadtxt('/home/lizq/win7share/TJM.txt', delimiter=",")  # mm 更新TJM

    def manual_calibrate_button_clicked(self):
        pass

    def add_calibrate_point_button_clicked(self):
        pass

    def finish_manual_calibrate_button_clicked(self):
        pass

    def calibrate_error_button_clicked(self):
        pass

    def follow_button_clicked(self):
        self.follow_thread.start()

    def puncture_test_button_clicked(self):
        pass

    def joint_right_button_7_clicked(self):
        pass

    def joint_left_button_7_clicked(self):
        pass

    def position_z_add_button_clicked(self):
        pass

    def orientation_z_add_button_clicked(self):
        pass

    def lineEdit_joint_out_copy_button_clicked(self):
        clipboard = QtWidgets.QApplication.clipboard()
        clipboard.setText(self.lineEdit_joint_out.text())

    def lineEdit_pose_out_copy_button_clicked(self):
        clipboard = QtWidgets.QApplication.clipboard()
        clipboard.setText(self.lineEdit_pose_out.text())

    def refresh_button_clicked(self):
        self.textEdit_calibrate.setText("waiting")

    # 多线程槽函数们
    def follow_waiting_slot(self):
        self.textEdit_calibrate.setText("waiting")

    def follow_distance_slot(self,distance):
        self.textEdit_calibrate.setText(str(distance))

class Follow_Thread(QtCore.QThread):
    waiting_signal = QtCore.pyqtSignal()
    distance_signal = QtCore.pyqtSignal(float)

    def __int__(self):
        super(Follow_Thread, self).__init__()

    def run(self):
        rate = rospy.Rate(50)  # smart servo 可以达到 20ms 50hz
        ton = np.loadtxt('/home/lizq/win7share/TON.txt', delimiter=",")  # mm 需要优化
        tno = np.linalg.inv(ton)
        tcp = list(window.tcp_pose)  # 米 只取姿态
        tgg = np.loadtxt('/home/lizq/win7share/TGG.txt', delimiter=",")
        while window.follow_button.isChecked():  # 等待NDI数据
            try:
                ndi = np.genfromtxt('/home/lizq/win7share/NDI.txt', delimiter=",")
                if math.isnan(ndi[1][0]):
                    self.waiting_signal.emit()  # 发射信号使主界面waiting
                else:
                    tmg = qc.quat2matrix(ndi[1].tolist())  # 被动刚体位姿
                    tmg = tmg.dot(tgg)  # 更正钢针位姿
                    tjg = window.TJM.dot(tmg)  # 将钢针针尖位置变换至基座坐标系下

                    # 发射针尖目标距离
                    tjo = qc.quat2matrix(tcp)
                    tjo[0:3][:, 3] *= 1000
                    tjn = tjo.dot(ton)
                    print tjn
                    print tjg
                    distance = pow(pow(tjg[0][3]-tjn[0][3],2)+pow(tjg[1][3]-tjn[1][3],2)+pow(tjg[2][3]-tjn[2][3],2),0.5)
                    self.distance_signal[float].emit(distance)

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

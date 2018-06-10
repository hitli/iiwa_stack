# -*- coding: utf-8 -*-

from mainwindow import Ui_MainWindow
from PyQt5 import QtWidgets
from PyQt5.QtCore import QTimer
import sys
import rospy
import quaternion_calculate as qc
from numpy.linalg import inv
from math import isnan
from geometry_msgs.msg import PoseStamped
from iiwa_msgs.msg import JointPosition
import matlab.engine
import img_rcc_rc


class Mywindow(QtWidgets.QMainWindow, Ui_MainWindow):
    def __init__(self):
        # 全局变量
        self.joint_position = "未连接"  # a1,a2,a3,a4,a5,a6,a7
        self.tcp_pose = "未连接"  # x,y,z,rx,ry,rz,rw

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
        self.read_state()

        #  启动定时器
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.read_state)
        self.timer.start(300)  # 300ms

        # while self.joint_position == "未连接":
        #     try:
        #         print "loading topics"
        #         joint_position_save = self.joint_position
        #     except:
        #         pass

    def read_state(self):
        self.lineEdit_pose_out.setText(self.tcp_pose)
        self.lineEdit_joint_out.setText(str(self.joint_position))
        self.progressBar_1.setProperty("value", self.joint_position[0])
        self.progressBar_2.setProperty("value", self.joint_position[1])
        self.progressBar_3.setProperty("value", self.joint_position[2])
        self.progressBar_4.setProperty("value", self.joint_position[3])
        self.progressBar_5.setProperty("value", self.joint_position[4])
        self.progressBar_6.setProperty("value", self.joint_position[5])
        self.progressBar_7.setProperty("value", self.joint_position[6])
        with open('/home/lizq/win7share/NDI.txt', 'r') as ndi:
            ndi = ndi.read().splitlines()
            self.lineEdit_ndi449.setText(ndi[0])
            self.lineEdit_ndi340.setText(ndi[1])
            self.lineEdit_ndi339.setText(ndi[2])

    def get_tcp_pose(self, tcp):
        self.tcp_pose = (tcp.pose.position.x, tcp.pose.position.y, tcp.pose.position.z, tcp.pose.orientation.x,tcp.pose.orientation.y, tcp.pose.orientation.z, tcp.pose.orientation.w)

    def get_joint_position(self, joint):
        self.joint_position = (joint.position.a1, joint.position.a2, joint.position.a3, joint.position.a4, joint.position.a5, joint.position.a6, joint.position.a7)

    # def btn_click(self):
    #     self.textEdit.setText("hi,PyQt5~")

    def joint_add_button_clicked(self):
        pass

    def joint_absolute_button_clicked(self):
        pass

    def pose_add_button_clicked(self):
        pass

    def pose_absolute_button_clicked(self):
        pass

    def auto_calibrate_button_clicked(self):
        calibrate_start_pose = self.tcp_pose


    def manual_calibrate_button_clicked(self):
        pass

    def add_calibrate_point_button_clicked(self):
        pass

    def finish_manual_calibrate_button_clicked(self):
        pass

    def calibrate_error_button_clicked(self):
        pass

    def follow_button_clicked(self):
        pass

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
        clipboard.setText(str(self.joint_position))

    def lineEdit_pose_out_copy_button_clicked(self):
        clipboard = QtWidgets.QApplication.clipboard()
        clipboard.setText(str(self.tcp_pose))

app = QtWidgets.QApplication(sys.argv)
window = Mywindow()
window.show()
sys.exit(app.exec_())

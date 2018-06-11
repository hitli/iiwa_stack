# -*- coding: utf-8 -*-

from mainwindow import Ui_MainWindow
from PyQt5 import QtWidgets
from PyQt5.QtCore import QTimer
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
        super(Mywindow, self).__init__()
        self.setupUi(self)

    def read_state(self):
        pass

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
        pass

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
        pass

    def lineEdit_pose_out_copy_button_clicked(self):
        pass

    def refresh_button_clicked(self):
        pass


app = QtWidgets.QApplication(sys.argv)
window = Mywindow()
window.show()
sys.exit(app.exec_())
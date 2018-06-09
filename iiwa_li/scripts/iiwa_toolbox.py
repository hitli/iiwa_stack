# -*- coding: utf-8 -*-

from mainwindow import Ui_MainWindow
from PyQt5 import QtWidgets
import sys
import img_rcc_rc


class Mywindow(QtWidgets.QMainWindow,Ui_MainWindow):
    def __init__(self):
        super(Mywindow, self).__init__()
        self.setupUi(self)

    def btn_click(self):
        self.textEdit.setText("hi,PyQt5~")

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


app = QtWidgets.QApplication(sys.argv)
window = Mywindow()
window.show()
sys.exit(app.exec_())
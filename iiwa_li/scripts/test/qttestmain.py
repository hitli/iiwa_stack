# -*- coding: utf-8 -*-
from qttest import Ui_MainWindow  # 导入uitestPyQt5.ui转换为uitestPyQt5.py中的类

from PyQt5 import QtWidgets
import sys


class Mywindow(QtWidgets.QMainWindow, Ui_MainWindow):
    # 建立的是Main Window项目，故此处导入的是QMainWindow
    # 参考博客中建立的是Widget项目，因此哪里导入的是QWidget
    def __init__(self):
        super(Mywindow, self).__init__()
        self.setupUi(self)

app = QtWidgets.QApplication(sys.argv)
window = Mywindow()
window.show()
sys.exit(app.exec_())
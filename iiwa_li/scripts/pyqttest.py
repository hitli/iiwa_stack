
# !/usr/bin/env python
# encoding: utf-8

import sys
from PyQt5 import QtCore, QtGui, QtWidgets

if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)
    w = QtWidgets.QWidget()
    w.resize(250, 150)
    w.move(300, 300)
    w.setWindowTitle('helloworld')
    w.show()
    sys.exit(app.exec_())

import sys
from PyQt5.QtWidgets import QApplication
from PyQt5.uic import loadUi
app = QApplication(sys.argv)
widget = loadUi('test.ui')
widget.show()
sys.exit(app.exec_())
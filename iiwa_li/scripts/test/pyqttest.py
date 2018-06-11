#!/usr/bin/python
# simple.py

import sys
from PyQt5 import QtWidgets

app = QtWidgets.QApplication(sys.argv)
widget = QtWidgets.QWidget()
widget.resize(250, 150)
widget.setWindowTitle('simple')
widget.show()
sys.exit(app.exec_())
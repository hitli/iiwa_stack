# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'client_window.ui'
#
# Created by: PyQt5 UI code generator 5.10.1
#
# WARNING! All changes made in this file will be lost!

from PyQt5 import QtCore, QtGui, QtWidgets

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(579, 411)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.verticalLayoutWidget_2 = QtWidgets.QWidget(self.centralwidget)
        self.verticalLayoutWidget_2.setGeometry(QtCore.QRect(0, 0, 573, 355))
        self.verticalLayoutWidget_2.setObjectName("verticalLayoutWidget_2")
        self.verticalLayout_3 = QtWidgets.QVBoxLayout(self.verticalLayoutWidget_2)
        self.verticalLayout_3.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout_3.setObjectName("verticalLayout_3")
        self.horizontalLayout_2 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_2.setObjectName("horizontalLayout_2")
        self.textEdit = QtWidgets.QTextEdit(self.verticalLayoutWidget_2)
        self.textEdit.setObjectName("textEdit")
        self.horizontalLayout_2.addWidget(self.textEdit)
        self.verticalLayout = QtWidgets.QVBoxLayout()
        self.verticalLayout.setObjectName("verticalLayout")
        self.keep_get_TMN_btn = QtWidgets.QPushButton(self.verticalLayoutWidget_2)
        self.keep_get_TMN_btn.setCheckable(True)
        self.keep_get_TMN_btn.setObjectName("keep_get_TMN_btn")
        self.verticalLayout.addWidget(self.keep_get_TMN_btn)
        self.get_TMN_btn = QtWidgets.QPushButton(self.verticalLayoutWidget_2)
        self.get_TMN_btn.setCheckable(False)
        self.get_TMN_btn.setObjectName("get_TMN_btn")
        self.verticalLayout.addWidget(self.get_TMN_btn)
        self.send_TMC_btn = QtWidgets.QPushButton(self.verticalLayoutWidget_2)
        self.send_TMC_btn.setObjectName("send_TMC_btn")
        self.verticalLayout.addWidget(self.send_TMC_btn)
        self.jinzhendian_btn = QtWidgets.QPushButton(self.verticalLayoutWidget_2)
        self.jinzhendian_btn.setObjectName("jinzhendian_btn")
        self.verticalLayout.addWidget(self.jinzhendian_btn)
        self.chuancidian_btn = QtWidgets.QPushButton(self.verticalLayoutWidget_2)
        self.chuancidian_btn.setObjectName("chuancidian_btn")
        self.verticalLayout.addWidget(self.chuancidian_btn)
        self.move_jinzhendian_btn = QtWidgets.QPushButton(self.verticalLayoutWidget_2)
        self.move_jinzhendian_btn.setObjectName("move_jinzhendian_btn")
        self.verticalLayout.addWidget(self.move_jinzhendian_btn)
        self.move_chuancidian_btn = QtWidgets.QPushButton(self.verticalLayoutWidget_2)
        self.move_chuancidian_btn.setObjectName("move_chuancidian_btn")
        self.verticalLayout.addWidget(self.move_chuancidian_btn)
        self.horizontalLayout_2.addLayout(self.verticalLayout)
        self.verticalLayout_3.addLayout(self.horizontalLayout_2)
        self.horizontalLayout = QtWidgets.QHBoxLayout()
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.label = QtWidgets.QLabel(self.verticalLayoutWidget_2)
        self.label.setObjectName("label")
        self.horizontalLayout.addWidget(self.label)
        self.lineEdit_ip = QtWidgets.QLineEdit(self.verticalLayoutWidget_2)
        self.lineEdit_ip.setObjectName("lineEdit_ip")
        self.horizontalLayout.addWidget(self.lineEdit_ip)
        self.label_2 = QtWidgets.QLabel(self.verticalLayoutWidget_2)
        self.label_2.setObjectName("label_2")
        self.horizontalLayout.addWidget(self.label_2)
        self.lineEdit_port = QtWidgets.QLineEdit(self.verticalLayoutWidget_2)
        self.lineEdit_port.setObjectName("lineEdit_port")
        self.horizontalLayout.addWidget(self.lineEdit_port)
        self.connect_btn = QtWidgets.QPushButton(self.verticalLayoutWidget_2)
        self.connect_btn.setObjectName("connect_btn")
        self.horizontalLayout.addWidget(self.connect_btn)
        self.stop_connect_btn = QtWidgets.QPushButton(self.verticalLayoutWidget_2)
        self.stop_connect_btn.setObjectName("stop_connect_btn")
        self.horizontalLayout.addWidget(self.stop_connect_btn)
        self.verticalLayout_3.addLayout(self.horizontalLayout)
        self.horizontalLayout_4 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_4.setObjectName("horizontalLayout_4")
        self.lineEdit = QtWidgets.QLineEdit(self.verticalLayoutWidget_2)
        self.lineEdit.setObjectName("lineEdit")
        self.horizontalLayout_4.addWidget(self.lineEdit)
        self.move_btn = QtWidgets.QPushButton(self.verticalLayoutWidget_2)
        self.move_btn.setObjectName("move_btn")
        self.horizontalLayout_4.addWidget(self.move_btn)
        self.verticalLayout_3.addLayout(self.horizontalLayout_4)
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 579, 31))
        self.menubar.setObjectName("menubar")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        self.get_TMN_btn.clicked.connect(MainWindow.get_TMN_btn_clicked)
        self.send_TMC_btn.clicked.connect(MainWindow.send_TMC_btn_clicked)
        self.jinzhendian_btn.clicked.connect(MainWindow.jinzhendian_btn_clicked)
        self.move_jinzhendian_btn.clicked.connect(MainWindow.move_jinzhendian_btn_clicked)
        self.move_chuancidian_btn.clicked.connect(MainWindow.move_chuancidian_btn_clicked)
        self.chuancidian_btn.clicked.connect(MainWindow.chuancidian_btn_clicked)
        self.connect_btn.clicked.connect(MainWindow.connect_btn_clicked)
        self.stop_connect_btn.clicked.connect(MainWindow.stop_connect_btn_clicked)
        self.move_btn.clicked.connect(MainWindow.move_btn_clicked)
        self.keep_get_TMN_btn.clicked.connect(MainWindow.keep_get_TMN_btn_clicked)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "模拟客户端"))
        self.keep_get_TMN_btn.setText(_translate("MainWindow", "持续接受针尖"))
        self.get_TMN_btn.setText(_translate("MainWindow", "接收针尖位置"))
        self.send_TMC_btn.setText(_translate("MainWindow", "发射标定向量"))
        self.jinzhendian_btn.setText(_translate("MainWindow", "进针点坐标"))
        self.chuancidian_btn.setText(_translate("MainWindow", "穿刺点坐标"))
        self.move_jinzhendian_btn.setText(_translate("MainWindow", "运动至进针点"))
        self.move_chuancidian_btn.setText(_translate("MainWindow", "运动至穿刺点"))
        self.label.setText(_translate("MainWindow", "IP"))
        self.lineEdit_ip.setText(_translate("MainWindow", "127.0.0.1"))
        self.label_2.setText(_translate("MainWindow", "port"))
        self.lineEdit_port.setText(_translate("MainWindow", "9999"))
        self.connect_btn.setText(_translate("MainWindow", "连接服务器"))
        self.stop_connect_btn.setText(_translate("MainWindow", "断开连接"))
        self.move_btn.setText(_translate("MainWindow", "运动至指定点"))

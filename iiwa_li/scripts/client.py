#coding:utf-8
from client_window import Ui_MainWindow
from PyQt5 import QtWidgets, QtCore, QtGui
import sys
import socket
import time


class Mywindow(QtWidgets.QMainWindow, Ui_MainWindow):

    def __init__(self):
        super(Mywindow, self).__init__()
        self.setupUi(self)
        self.sk = socket.socket()

    def connect_btn_clicked(self):
        ip = self.lineEdit_ip.text()
        port = int(self.lineEdit_port.text())
        try:
            self.sk.connect(('127.0.0.1', 9999))
        except Exception as e:
            print e
            self.textEdit.append("连接失败")
        else:
            self.textEdit.append("已连接服务器")

    def stop_connect_btn_clicked(self):
        try:
            self.sk.close()
        except:
            pass
        else:
            self.textEdit.append("已断开连接")

    def get_TMN_btn_clicked(self):
        # while self.get_TMN_btn.isChecked():
        self.sk.sendall("请求针尖位置")
        TMN = self.sk.recv(1024)
        self.textEdit.append(str(TMN))
        print TMN
        # time.sleep(0.5)

    def send_TMC_btn_clicked(self):
        TMC = (137.5000,192.4200,-1686.9700,0.1686,-0.5152,0.2581,0.7996)
        self.sk.sendall(str(TMC))
        sentence = "发送标定向量\n" + str(TMC)
        self.textEdit.append(sentence)
        repeat = self.sk.recv(1024)
        if repeat == "收到命令":
            self.textEdit.append("服务器已收到")
        else:
            self.textEdit.append("发送失败")

    def jinzhendian_btn_clicked(self):
        p1 = (-40.4500,289.9500,-1615.9700,0.1683,-0.5151,0.2576,0.7999)
        sentence = "发送进针点\n"+str(p1)
        self.textEdit.append(sentence)
        self.sk.sendall(sentence)
        repeat = self.sk.recv(1024)
        if repeat == "收到命令":
            self.textEdit.append("服务器已收到")
        else:
            self.textEdit.append("发送失败")

    def chuancidian_btn_clicked(self):
        p1 = (-44.3000,276.4300,-1610.8900,0.1684,-0.5144,0.2577,0.8003)
        sentence = "发送穿刺点\n"+str(p1)
        self.textEdit.append(sentence)
        self.sk.sendall(sentence)
        repeat = self.sk.recv(1024)
        if repeat == "收到命令":
            self.textEdit.append("服务器已收到")
        else:
            self.textEdit.append("发送失败")

    def move_jinzhendian_btn_clicked(self):
        self.sk.sendall("运动至进针点")
        self.textEdit.append("运动至进针点")
        repeat = self.sk.recv(1024)
        if repeat == "收到命令":
            self.textEdit.append("服务器已收到")
        else:
            self.textEdit.append("发送失败")

    def move_chuancidian_btn_clicked(self):
        self.sk.sendall("运动至穿刺点")
        self.textEdit.append("运动至穿刺点")
        repeat = self.sk.recv(1024)
        if repeat == "收到命令":
            self.textEdit.append("服务器已收到")
        else:
            self.textEdit.append("发送失败")


app = QtWidgets.QApplication(sys.argv)
window = Mywindow()
window.show()
sys.exit(app.exec_())
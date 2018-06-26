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

        self.get_TMN_thread = Get_TMN_Thread()
        self.get_TMN_thread.append_signal[str].connect(self.append_slot)

    def connect_btn_clicked(self):
        try:
            self.sk = socket.socket()
            ip = self.lineEdit_ip.text()
            port = int(self.lineEdit_port.text())
            self.sk.connect((ip,port))
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
        self.sk.sendall("ask for tmn")
        TMN = self.sk.recv(1024)
        self.textEdit.append(str(TMN))
        print TMN
        # time.sleep(0.5)

    def keep_get_TMN_btn_clicked(self):
        self.get_TMN_thread.start()

    def send_TMC_btn_clicked(self):
        TMC = (137.5000,192.4200,-1686.9700,0.1686,-0.5152,0.2581,0.7996)
        sentence = "send tmc quat\n" + str(TMC)
        self.sk.sendall(sentence)
        self.textEdit.append(sentence)
        reply = self.sk.recv(1024)
        self.textEdit.append(reply)

    def move_btn_clicked(self):
        point = (-40.4500,289.9500,-1615.9700,0.0,0.0,0.0,1.0)
        sentence = "move to point\n" + self.lineEdit.text()
        self.sk.sendall(sentence)
        self.textEdit.append(sentence)
        reply = self.sk.recv(1024)
        self.textEdit.append(reply)

    def jinzhendian_btn_clicked(self):
        p1 = (-40.4500,289.9500,-1615.9700,0.0,0.0,0.0,1.0)
        sentence = "send entry point\n"+str(p1)
        self.textEdit.append(sentence)
        self.sk.sendall(sentence)
        reply = self.sk.recv(1024)
        self.textEdit.append(reply)

    def chuancidian_btn_clicked(self):
        p1 = (-44.3000,276.4300,-1610.8900,0.0,0.0,0.0,1.0)
        sentence = "send puncture point\n"+str(p1)
        self.textEdit.append(sentence)
        self.sk.sendall(sentence)
        reply = self.sk.recv(1024)
        self.textEdit.append(reply)

    def move_jinzhendian_btn_clicked(self):
        self.sk.sendall("move to entry point")
        self.textEdit.append("move to entry point")
        reply = self.sk.recv(1024)
        self.textEdit.append(reply)

    def move_chuancidian_btn_clicked(self):
        self.sk.sendall("move to puncture point")
        self.textEdit.append("move to puncture point")
        reply = self.sk.recv(1024)
        self.textEdit.append(reply)

    def append_slot(self,str):
        self.textEdit.append(str)


class Get_TMN_Thread(QtCore.QThread):
    append_signal = QtCore.pyqtSignal(str)

    def __int__(self):
        super(Get_TMN_Thread, self).__init__()

    def run(self):
        while window.keep_get_TMN_btn.isChecked():
            window.sk.sendall("ask for tmn")
            TMN = window.sk.recv(1024)
            self.append_signal[str].emit(TMN)
            time.sleep(1)
            print TMN




app = QtWidgets.QApplication(sys.argv)
window = Mywindow()
window.show()
sys.exit(app.exec_())
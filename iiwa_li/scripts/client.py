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

    def tmg_btn_clicked(self):
        self.sk.sendall("ask for tmg")
        TMG = self.sk.recv(1024)
        self.textEdit.append(str(TMG))
        print TMG

    def keep_get_TMN_btn_clicked(self):
        self.get_TMN_thread.start()

    def move_btn_clicked(self):
        point = (-40.4500,289.9500,-1615.9700,0.0,0.0,0.0,1.0)
        sentence = "move to point\n" + self.lineEdit.text()
        self.sk.sendall(sentence)
        self.textEdit.append(sentence)
        reply = self.sk.recv(1024)
        self.textEdit.append(reply)

    def path_btn_clicked(self):
        path = (-74.214,111.528,-1383.14,-31.1147,103.454,-1415.52)
        # path = (47.1193,130.8626,-1313.0105,64.4142,152.6892,-1317.6314)
        # path = (83.6979,-28.5724,-1137.6173,133.6672,-10.0353,-1153.9324)
        # path = (61.7195,-32.8224,-1352.2596,78.1596,-32.9236,-1353.8509)  # 0725
        path = (152.419,36.289,-1398.56,198.309,43.6798,-1427.03)  # 0726实验一
        sentence = "send path\n"+str(path)
        self.textEdit.append(sentence)
        self.sk.sendall(sentence)
        reply = self.sk.recv(1024)
        self.textEdit.append(reply)

    def change_pose_btn_clicked(self):
        self.sk.sendall("change pose")
        self.textEdit.append("change pose")
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
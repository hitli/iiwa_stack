# -*- coding: utf-8 -*-
import socket
import time

if __name__ == '__main__':
    server = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
    server.bind(('192.168.168.10',9999))
    server.listen(0)
    while True:
        connection,address = server.accept()
        # print connection,address
        recv_str = connection.recv(1024)
        print recv_str,type(recv_str)
        # recv_str1 = recv_str.decode("ascii")
        if "move" in recv_str:
            print "yes"
        recv_str1 = recv_str.splitlines()[1]
        print recv_str1,type(recv_str1)

        data = eval(recv_str1)
        print data
        # connection.send(bytes("test: %s" %recv_str,encoding="ascii"))
        connection.send("command received")
        time.sleep(0.5)
    connection.close()
    input()
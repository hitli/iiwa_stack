#coding:utf-8
import socket
import sys

# 链接服务端ip和端口
ip_port = ('127.0.0.1',9999)

# 生成一个句柄
sk = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
try:
    # 请求连接服务端
    sk.connect(ip_port)
except:
    print '服务器没开'
    sys.exit()
while True:
    trigger = raw_input("send:")
    sk.sendall(str(trigger))
    data = sk.recv(1024)
    print str(data)
    if trigger.lower() == '1':#发送1结束连接
        break
sk.close()
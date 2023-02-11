#!/usr/bin/env python3


import socket


HOST = '127.0.0.1'  # The server's hostname or IP address
PORT = 3200        # The port used by the server

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.connect((HOST, PORT))
    # s.sendall(b'Hello, world')
    s.recv(10000)
    s.recv(10000)
    s.recv(10000)
    while True:
        msg = "(playMode PlayOn)"
        msg = str(len(msg)) + msg
        s.send(str.encode(msg))
        # data = s.recv(10000)
        # print('Received', repr(data))


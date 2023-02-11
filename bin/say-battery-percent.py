#!/usr/bin/python

import os
import socket
import msgpack

def open():
    os.system("/usr/bin/pkill --signal 9 say.py")
    s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    s.connect("/tmp/robocup") # This is some sort of socket file ?
    return s

def close(s):
    s.close()

def get_data(s):
    stream = s.recv(896)
    upacker = msgpack.unpackb(stream)
    return upacker

def get_battery_level(upacker_data):
    battery = dict(zip(["Charge", "Status", "Current", "Temperature"], upacker_data["Battery"]))
    return battery["Charge"]

if __name__ == "__main__":
    s = open()
    print("Battery level: ", get_battery_level(get_data(s)))
    close()

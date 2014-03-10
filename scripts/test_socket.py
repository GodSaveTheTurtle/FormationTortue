#!/usr/bin/env python

import socket

if __name__ == '__main__':
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(('0.0.0.0', 1337))

    x, y = 0, 0

    while True:
        data = s.recv(1024).split(" ")
        x, y = [int(i) for i in data]
        print x*3, y*2

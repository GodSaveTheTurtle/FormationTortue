#!/usr/bin/env python

import socket


if __name__ == '__main__':
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(('0.0.0.0', 1337))

    while True:
        data = s.recv(1024)
        print data

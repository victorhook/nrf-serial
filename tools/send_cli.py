#!/usr/bin/env python3

from serial import Serial
import sys
import time
import struct


if __name__ == '__main__':
    port = sys.argv[1]
    baud = 115200
    s = Serial(port, baudrate=baud)
    s.flush()

    while True:
        data = input('> ')
        s.write(data.encode('ascii'))
#!/usr/bin/env python3

from serial import Serial
import sys
import struct
import time

if __name__ == '__main__':
    port = sys.argv[1]
    baud = 921600
    s = Serial(port, baudrate=baud)
    s.flush()
    last_nbr = -1
    bytes_received = 0
    bytes_per_sec_counter = 0
    dropped_data = 0
    t0 = time.time()
    fmt = 'B'

    while True:
        data = struct.unpack(fmt, s.read(struct.calcsize(fmt)))[0]
        bytes_received += struct.calcsize(fmt)
        bytes_per_sec_counter += struct.calcsize(fmt)
        #print(ord(data))


        if (time.time() - t0) > 1:
            print(f'Datarate: {bytes_per_sec_counter} B/s (total: {bytes_received})')
            bytes_per_sec_counter = 0
            t0 = time.time()

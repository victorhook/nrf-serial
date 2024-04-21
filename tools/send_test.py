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

    delay = 1
    i = 0
    TX_SIZE = 100
    bytes_sent = 0
    bytes_per_sec_counter = 0
    t0 = time.time()

    while True:
        data = bytearray()
        for j in range(TX_SIZE):
            data.append(i)
            i = (i + 1) % 255

        bytes_sent += TX_SIZE
        bytes_per_sec_counter += TX_SIZE
        s.write(data)
        time.sleep(delay)


        if (time.time() - t0) > 1:
            print(f'Datarate: {bytes_per_sec_counter} B/s (total: {bytes_sent})')
            bytes_per_sec_counter = 0
            t0 = time.time()
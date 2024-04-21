#!/usr/bin/env python3

from serial import Serial
import sys
import time
import struct


if __name__ == '__main__':
    port = sys.argv[1]
    file = sys.argv[2]
    baud = 115200
    s = Serial(port, baudrate=baud)
    s.flush()

    chunksize = 1000
    delay = 0.01
    print(f'Sending file {file}, chunksize: {chunksize}, delay: {delay}')
    sent = 0
    t0 = time.time()

    with open(file, 'rb') as f:
        f.seek(0, 2)
        end_of_file = f.tell()
        f.seek(0, 0)

        while sent < end_of_file:
            bytes_left = end_of_file - sent
            if bytes_left > chunksize:
                bufsize = chunksize
            else:
                bufsize = bytes_left

            s.write(f.read(bufsize))
            sent += bufsize

            perc = (sent / end_of_file) * 100
            sys.stdout.write(f'\r{int(perc)}% ({sent} B sent)')
            sys.stdout.flush()
            time.sleep(delay)

    dt = time.time() - t0
    dr = sent / dt
    print(f'\nDone, after {round(dt, 2)} s, average ({int(dr)} B/s)')
#!/usr/bin/env python3

from threading import Thread
from serial import Serial
import sys
import struct
import time
from queue import Queue
from threading import Semaphore

TX = '/dev/ttyUSB1'
RX = '/dev/ttyACM1'

rx_expect = Queue()
tx_sent = Queue()
errors = 0
bytes_sent = 0
bytes_received = 0
delay = 0.1
TX_SIZE = 100
sem = Semaphore(1)

def reader():
    s = Serial(RX, baudrate=115200)
    s.flush()
    global errors
    global rx_expect
    global bytes_received
    while True:
        byte = s.read(1)
        expected = rx_expect.get()
        bytes_received += 1
        if ord(byte) != expected:
            #print(ord(byte), expected)
            #sem.acquire()
            #rx_expect = Queue()
            #s.flush()
            #sem.release()
            #print(ord(byte), ord(expected))
            errors += 1

def writer():
    s = Serial(TX, baudrate=115200)
    s.flush()
    i = 0
    global sent
    global TX_SIZE
    global bytes_sent
    while True:
        data = bytearray()
        for j in range(TX_SIZE):
            data.append(i)
            i = (i + 1) % 255

        bytes_sent += TX_SIZE
        for d in data:
            rx_expect.put(d)
        s.write(data)
        time.sleep(delay)


if __name__ == '__main__':
    Thread(target=reader, daemon=True).start()
    Thread(target=writer, daemon=True).start()

    t0 = time.time()
    last_sent = 0
    last_rx = 0
    while True:
        if (time.time() - t0) > 1:
            ds = bytes_sent - last_sent
            dr = bytes_received - last_rx
            print(f'{dr}/{ds} B/s (errors: {errors})')
            last_rx = bytes_received
            last_sent = bytes_sent
            t0 = time.time()
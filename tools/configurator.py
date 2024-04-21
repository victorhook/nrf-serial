#!/usr/bin/env python3

import struct
from dataclasses import dataclass
from subprocess import Popen, PIPE
import os
import time
from serial import Serial
from pathlib import Path
from tkinter import ttk
import tkinter as tk
from threading import Thread
import sys


ROOT_DIR = Path(__file__).absolute().parent
SETTINGS_START_ADDRESS = 0x101ff000


@dataclass
class Settings:
    tx_address: bytes    # uint8_t[5]
    rx_address1: bytes   # uint8_t[5]
    rx_address2: bytes   # uint8_t[5]
    rx_address3: bytes   # uint8_t[5]
    rx_address4: bytes   # uint8_t[5]
    rx_address5: bytes   # uint8_t[5]
    radio_pa_level: int  # uint8_t
    radio_channel: int   # uint8_t
    radio_datarate: int  # uint8_t
    serial_baudrate: int # uint32_t

    _FMT = '<' + ''.join([
        '5s',
        '5s',
        '5s',
        '5s',
        '5s',
        '5s',
        'B',
        'B',
        'B',
        'I',
    ])

    def to_bytes(self) -> bytes:
        return struct.pack(self._FMT, *[getattr(self, key) for key in self.__dataclass_fields__.keys()])

    @classmethod
    def size(cls) -> int:
        return struct.calcsize(cls._FMT)

    @classmethod
    def from_bytes(cls, raw: bytes) -> 'Settings':
        return Settings(*struct.unpack(cls._FMT, raw))


class Device:

    def __init__(self) -> None:
        self._is_in_bootsel = False
        self._settings_start_addr = SETTINGS_START_ADDRESS
        self._tmp_file = str(ROOT_DIR.joinpath('.tmp_settings.bin'))

    def read_settings(self) -> Settings:
        '''
        Reads the active settings on the device.
        Note that the device must be in bootsel mode for this to work.
        '''

        start = hex(self._settings_start_addr)
        end = hex(self._settings_start_addr + Settings.size())
        self._cmd(f'picotool save -r {start} {end} {self._tmp_file}')

        # Picotool stores the data to a file, so we'll load it from that here
        with open(self._tmp_file, 'rb') as f:
            settings = Settings.from_bytes(f.read())

        # Clean up the temp file once we're done
        #os.remove(self._tmp_file)

        return settings

    def write_settings(self, settings: Settings) -> None:
        '''
        Writes the given settings to the device.
        Note that the device must be in bootsel mode for this to work.
        '''

        # First we'll write the settings to a file, so that picotool can load it
        with open(self._tmp_file, 'wb') as f:
            f.write(settings.to_bytes())

        start = hex(self._settings_start_addr)
        self._cmd(f'picotool load -v {self._tmp_file} -o {start}')

    def reboot(self) -> None:
        if not self._is_in_bootsel:
            print('Device is not in bootsel mode, so cannot reboot!')
            return
        self._cmd('picotool reboot')
        self._is_in_bootsel = False

    def put_into_bootsel(self, port: str) -> None:
        if self._is_in_bootsel:
            print('Device is already in bootsel mode!')
            return
        try:
            com = Serial(port, baudrate=1200, dsrdtr=True)
            com.dtr = False
            com.rts = False
            time.sleep(0.2)
            com.close()
        except OSError:
            # Typically we get "OSError: [Errno 71] Protocol error", but no worries
            pass

        self._is_in_bootsel = True

    def is_in_bootsel(self) -> bool:
        return self._is_in_bootsel

    def _cmd(self, cmd: str) -> str:
        p = Popen(cmd, shell=True, stdout=PIPE, stderr=PIPE)
        out = p.stdout.read().decode('ascii')
        err = p.stderr.read().decode('ascii')
        if out:
            print(out)
        if err:
            print(f'ERROR: {err}')
        return out


BACKGROUND = 'white'


class Gui(tk.Tk):


    def __init__(self) -> None:
        super().__init__()
        self.title('NRF-Serial Configurator')
        self.configure(background=BACKGROUND)

        self.frame_connect = tk.Frame(self, background=BACKGROUND)
        self.frame_settings = tk.Frame(self, background=BACKGROUND)
        self.frame_control = tk.Frame(self, background=BACKGROUND)

        pad = {'padx': 10, 'pady': 10}
        self.frame_connect.pack(fill=tk.X, **pad)
        self.frame_settings.pack(fill=tk.BOTH, expand=True, **pad)
        self.frame_control.pack(fill=tk.X, **pad)

        # -- Variables -- #
        self.var_connect = tk.StringVar()
        self.var_connect.set('Connect')
        self.serial_ports = []
        self.device = Device()

        # -- Connect frame -- #
        tk.Label(self.frame_connect, text='Serial port', background=BACKGROUND).grid(row=0, column=0)
        self.port_dropdown = ttk.Combobox(self.frame_connect, values=self.serial_ports, state='readonly')
        self.port_dropdown.grid(row=1, column=0)
        tk.Button(self.frame_connect, textvariable=self.var_connect, command=self._connect, background=BACKGROUND).grid(row=1, column=1, padx=20)

        # -- Settings frame -- #
        row = 0
        col = 0
        default_addr = '00 00 00 00 00'

        self.tx_address = self.Parameter(self.frame_settings, 'TX Address', default_addr, row + 0, col, 'Space separated hex (5)')
        self.rx_address1 = self.Parameter(self.frame_settings, 'RX Address 1', default_addr, row + 1, col, 'Space separated hex (5)')
        self.rx_address2 = self.Parameter(self.frame_settings, 'RX Address 2', default_addr, row + 2, col, 'Space separated hex (5)')
        self.rx_address3 = self.Parameter(self.frame_settings, 'RX Address 3', default_addr, row + 3, col, 'Space separated hex (5)')
        self.rx_address4 = self.Parameter(self.frame_settings, 'RX Address 4', default_addr, row + 4, col, 'Space separated hex (5)')
        self.rx_address5 = self.Parameter(self.frame_settings, 'RX Address 5', default_addr, row + 5, col, 'Space separated hex (5)')
        self.radio_pa_level = self.Parameter(self.frame_settings, 'PA Level', '0', row + 6, col, '0=MIN, 1=LOW, 2=HIGH, 3=MAX')
        self.radio_channel = self.Parameter(self.frame_settings, 'Channel', '0', row + 7, col, '0-125')
        self.radio_datarate = self.Parameter(self.frame_settings, 'Datarate', '0', row + 8, col, '0=1MBps, 1=2MBps, 2=250kbps')
        self.serial_baudrate = self.Parameter(self.frame_settings, 'Serial baudrate', '115200', row + 9, col, 'Baud rate to use for serial')

        # -- Control frame -- #
        self.console = tk.Text(self.frame_control, height=10, width=80)
        self.btn_read = tk.Button(self.frame_control, text='Read settings', command=self._read, background=BACKGROUND)
        self.btn_write = tk.Button(self.frame_control, text='Write settings', command=self._write, background=BACKGROUND)
        self.console.grid(rowspan=2, column=0, padx=15)
        self.btn_read.grid(row=0, column=1)
        self.btn_write.grid(row=1, column=1)

        # Before we'll start we'll hijack sys stdout a bit to also print it to tk console
        self.sys_stdout_write = sys.stdout.write
        self.sys_stderr_write = sys.stderr.write

        def _stdout_write(data: str):
            self.sys_stdout_write(data)
            self.console.insert(tk.END, data)
            self.console.see(tk.END)  # Scroll to the end

        def _stderr_write(data: str):
            self.sys_stdout_write(data)
            self.console.insert(tk.END, 'ERROR\n')
            self.console.insert(tk.END, data)
            self.console.insert(tk.END, 'ERROR\n')
            self.console.see(tk.END)  # Scroll to the end


        sys.stdout.write = _stdout_write
        sys.stderr.write = _stderr_write

        Thread(target=self._serial_port_thread, daemon=True).start()

        self._update_ui()

    class Parameter:

        def __init__(self, master, title: str, value: str, row: int, col: int, description: str, entry_width: int = 20) -> None:
            self.var = tk.StringVar()
            self.var.set(value)
            pad = {'padx': 5, 'pady': 2}
            tk.Label(master, text=title, background=BACKGROUND).grid(row=row, column=col, sticky=tk.W, **pad)
            tk.Entry(master, textvariable=self.var, width=entry_width).grid(row=row, column=col + 1, **pad)
            tk.Label(master, text=description, background=BACKGROUND).grid(row=row, column=col + 2, sticky=tk.W, **pad)

    def _connect(self) -> None:
        if self.device.is_in_bootsel():
            self.device.reboot()
        else:
            self.device.put_into_bootsel(self.port_dropdown.get())

        self._update_ui()

    def _read(self) -> None:
        settings = self.device.read_settings()

        def _addr(raw: bytes) -> str:
            return ' '.join(hex(b)[2:] for b in raw)

        self.tx_address.var.set(_addr(settings.tx_address))
        self.rx_address1.var.set(_addr(settings.rx_address1))
        self.rx_address2.var.set(_addr(settings.rx_address2))
        self.rx_address3.var.set(_addr(settings.rx_address3))
        self.rx_address4.var.set(_addr(settings.rx_address4))
        self.rx_address5.var.set(_addr(settings.rx_address5))
        self.radio_pa_level.var.set(str(settings.radio_pa_level))
        self.radio_channel.var.set(str(settings.radio_channel))
        self.radio_datarate.var.set(str(settings.radio_datarate))
        self.serial_baudrate.var.set(str(settings.serial_baudrate))

    def _write(self) -> None:
        def _addr(raw: str) -> bytes:
            return bytes(int(hexstr, 16) for hexstr in raw.split(' '))

        settings = Settings(
            _addr(self.tx_address.var.get()),
            _addr(self.rx_address1.var.get()),
            _addr(self.rx_address2.var.get()),
            _addr(self.rx_address3.var.get()),
            _addr(self.rx_address4.var.get()),
            _addr(self.rx_address5.var.get()),
            int(self.radio_pa_level.var.get()),
            int(self.radio_channel.var.get()),
            int(self.radio_datarate.var.get()),
            int(self.serial_baudrate.var.get())
        )
        
        print('Writing settings:')
        print(settings)

        self.device.write_settings(settings)

    def _update_ui(self) -> None:
        if self.device.is_in_bootsel():
            self.var_connect.set('Disconnect')
            self.btn_read.config(state='normal')
            self.btn_write.config(state='normal')
        else:
            self.var_connect.set('Connect')
            self.btn_read.config(state='disabled')
            self.btn_write.config(state='disabled')

    def _serial_port_thread(self) -> None:
        while True:
            if not self.device.is_in_bootsel():
                # If we're not connected, we'll check if new ports have been connected
                from serial.tools.list_ports import comports
                ports = [p.device for p in comports()]
                if ports != self.serial_ports:
                    self.serial_ports = ports
                    self.port_dropdown.config(values=self.serial_ports)
                    self.port_dropdown.current(0)
            time.sleep(1)

if __name__ == '__main__':
    gui = Gui()
    gui.mainloop()
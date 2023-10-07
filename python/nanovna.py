#!/usr/bin/env python3
import serial
import numpy as np
import struct
from cobs import cobs
from serial.tools import list_ports

VID = 0x0483 #1155
PID = 0x5740 #22336

# Get nanovna device automatically
def getport() -> str:
    device_list = list_ports.comports()
    for device in device_list:
        if device.vid == VID and device.pid == PID:
            return device.device
    raise OSError("device not found")

class NanoVNA:
    def __init__(self, dev = None):
        self.dev = dev or getport()
        self.serial = None
        
    def open(self):
        if self.serial is None:
            self.serial = serial.Serial(self.dev, timeout=2)
            self.serial.flush()
    def close(self):
        if self.serial:
            self.serial.close()
        self.serial = None

    def send_command(self, cmd):
        self.open()
        self.serial.write(cmd)
        data = []
        while 1:
            byte = self.serial.read(1)
            if (len(byte) == 0):
                print("Time out:")
                print(''.join(format(x, '02x') for x in data))
                # time-out
                return
            if byte == b'\x00':
                data = cobs.decode(bytearray(data))
                print("Ok:")
                print(''.join(format(x, '02x') for x in data))
                return
            else:
                data += byte
                
if __name__ == '__main__':
    from optparse import OptionParser
    parser = OptionParser(usage="%prog: [options]")
    parser.add_option("-P", "--port", type="int", dest="port",
                      help="port", metavar="PORT")
    parser.add_option("-d", "--dev", dest="device",
                      help="device node", metavar="DEV")
    (opt, args) = parser.parse_args()

    nv = NanoVNA(opt.device or getport())

    data = cobs.encode(b'\x01')
    data = data + b'\x00'

    print("Sending get callback count:")
    print(' '.join(format(x, '02x') for x in data))
    nv.send_command(data)

    data = cobs.encode(b'\x01')
    data = data + b'\x00'

    print("Sending get raw buffer:")
    print(' '.join(format(x, '02x') for x in data))
    nv.send_command(data)    


#!/usr/bin/python
# coding:utf-8

import serial
import time

class ultrasound(object):

    def __init__(self,port = None):
        # 超声波模块信息类
        if not port:
            self.port = '/dev/ttyS1'
        else:
            self.port = port
        self.ser = serial.Serial(self.port, 9600, timeout=0.5, write_timeout=0.5)
        # 获取四个探头数据的命令
        # self.send_cmd = [0x55, 0xAA, 0x01, 0x01, 0x07, 0xdf, 0x01, 0x2d, 0x00, 0x00, 0x11, 0x20, 0x46]
        self.send_cmd = [0x55, 0xAA, 0x01, 0x01, 0x07, 0xdf, 0x07, 0xdf, 0x07, 0xdf, 0x07, 0xdf, 0x99]
        sum_d = -self.send_cmd[-1]
        for i in self.send_cmd: sum_d += i
        print(sum_d)

        # 信息头
        self.head = [0x55, 0xAA, 0x01, 0x01]
        # 信息头之和
        self.sum_head=0
        for i in self.head: self.sum_head += i

    def get_data(self):
        self.send_data()
        return self.read_data()

    def send_data(self):
        self.ser.reset_input_buffer()
        if self.ser.is_open:
            self.ser.write(self.send_cmd)
        else:
            self.ser.open()
            self.ser.write(self.send_cmd)

    def read_data(self):
        # 读取四个超声波探头测得的数据
        timeout = 2.0
        pre_t = time.time()
        head_flag = False
        while True:
            for i in self.head:
                # 读取数据头
                while not self.ser.in_waiting:
                    # 超时返回
                    now = time.time()
                    if now - pre_t >= timeout:
                        break
                if self.ser.in_waiting:
                    head_byte = ord(self.ser.read())
                    if head_byte == i:
                        print(head_byte)
                        head_flag = True
                    else:
                        head_flag = False
                        break
            if head_flag:
                self.send_data()
            time.sleep(1)


if __name__ == '__main__':
    print('start:')
    dev = ultrasound('/dev/ttyUSB0')
    dev.read_data()

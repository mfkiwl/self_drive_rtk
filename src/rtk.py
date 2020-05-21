#coding=utf-8

import sys
import math
from math import sin, cos
import time
import serial
import threading
import numpy as np
import ntripClient
from redisHandler import redisHandler


class WGS84():
    def __init__(self, lon, lat, h):
        # WGS84坐标系参数
        self.a = 6378137         # 地球长轴半径
        self.f = 1/298.257223563  # 匾率
        self.b = self.a - self.a * self.f       # 短轴半径
        self.sqr_e = (math.pow(self.a,2)-math.pow(self.b, 2)) / math.pow(self.a, 2)
        self.h = h   # 高程
        self.p = self.calc_xyz(lon, lat, self.h)    # 坐标原点
        self.lon = math.radians(lon)
        self.lat = math.radians(lat)
        self.rotation = self.euler_to_rotation(0.0, math.pi/2-self.lat, self.lon, 'rzyx')
        print(self.rotation)
        self.translation = -self.p

    
    def euler_to_rotation(self, roll, pitch, yaw, mod = 'rzyx'):
        # 绕x旋转
        r_x = np.array([
            [1, 0, 0],
            [0, cos(roll), -sin(roll)],
            [0, sin(roll), cos(roll)]
            ])
        # 绕y旋转
        r_y = np.array([
            [cos(pitch), 0, sin(pitch)],
            [0, 1, 0],
            [-sin(pitch),0, cos(pitch)]
            ])
        # 绕z旋转
        r_z = np.array([
            [cos(yaw), -sin(yaw), 0],
            [sin(yaw), cos(yaw), 0],
            [0, 0, 1]
            ])
        if mod == 'rzyx':
            r = np.dot(r_z, np.dot(r_y, r_x))
        else:
            r = None
        return r

    def calc_xyz(self, lon, lat, h):
        # 中国x<0, y>0, z>0
        # print(lon,lat, h)
        lon = math.radians(lon)
        lat = math.radians(lat)
        n = self.a / math.sqrt(1 - self.sqr_e * math.sin(lat))
        x = (n + h) * math.cos(lat) * math.cos(lon)
        y = (n + h) * math.cos(lat) * math.sin(lon)
        z = (n * (1 - self.sqr_e) + h) * math.sin(lat)
        p = np.array([x, y, z]).T
        return p


class rtk(redisHandler):
    def init(self, port = '/dev/ttyS5', bps = 115200, timeout=2):
        self.pub_topics = ['rtk_out']
        self.base_lon = 121.24554536
        self.base_lat = 30.87471633
        self.base_h = 27.427
        self.wgs84 = WGS84(self.base_lon, self.base_lat, self.base_h)
        self.__ser = serial.Serial(port, bps, timeout=timeout)
        self.__position = {
                'p':None,
                'angle': 0.0,
                'precision': 0.0,
                'angle_precision':0.0,
                'rtcm':'',
                'rtk_mod':0
                }
        self.start_sub()
        self.__p_rtcm = threading.Thread(target=ntripClient.main_loop)
        self.__p_rtcm.setDaemon(True)
        self.__p_rtcm.start()


    def run(self):
        # rtk
        self.init()
        ser = self.__ser
        hz = 3  # 发送频率
        pre_time = time.time()
        while True:
            try:
                if ser.in_waiting:
                    #print('s')
                    data = ser.read_until('\n')
                    # print(data)
                    
                    # 报文处理
                    if 'GPGGA' in data:
                        data = data.split(',')
                        # print(data)
                        self.__position['rtk_mod'] = int(data[6])
                        # rtk 差分数据源传入时间间隔
                        self.__position['rtcm'] = data[13]
                        # 经度dddmm.mmmmm转度
                        # 纬度ddmm.mmmmm转度
                        lon = float(data[4]) / 100.0
                        lat = float(data[2]) / 100.0
                        h = float(data[9])
                        lon = int(lon) + (lon - int(lon)) * 100 / 60.0
                        lat = int(lat) + (lat - int(lat)) * 100 / 60.0
                        # 经纬度转xyz
                        self.__position['p'] = self.wgs84.calc_xyz(lon, lat, h)
                        # 精度因子
                        self.__position['precision'] = float(data[8])
                        #　坐标转换-->基站坐标系
                        p = self.__position['p']
                        p = p + self.wgs84.translation
                        # print(p.shape, self.wgs84.rotation.shape)
                        self.__position['p'] =np.dot(p, self.wgs84.rotation).tolist()
                        # print(self.__position)
                        # redis 发布
                        now = time.time()
                        if now - pre_time >= 1.0 / hz:
                            pre_time = now
                            pub_data = {
                                    'header': 'rtk_position',
                                    'data':self.__position
                                    }
                            self.pub_all(pub_data)
                    elif 'HEADINGA' in data:
                        data = data.split(';')[1]
                        data = data.split(',')
                        self.__position['angle'] = math.radians(360 - float(data[3]))
                        self.__position['precision_angle'] = float(data[6])
            except Exception as e:
                print('rtk err')
                print(e)





if __name__ == '__main__':
    rtk_rover = rtk()
    rtk_rover.run()

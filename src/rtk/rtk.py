#coding=utf-8

import math
import time
import serial
import pyproj
import threading
from multiprocessing import Queue
import keyboard
import numpy as np
from tf import transformations
import pygame
import ntripClient


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
        self.rotation = transformations.euler_matrix(self.lon, math.pi/2-self.lat, 0.0, 'rzyx')
        self.translation = -self.p

        
    def calc_xyz(self, lon, lat, h):
        # 中国x<0, y>0, z>0
        # print(lon,lat, h)
        lon = math.radians(lon)
        lat = math.radians(lat)
        n = self.a / math.sqrt(1 - self.sqr_e * math.sin(lat))
        x = (n + h) * math.cos(lat) * math.cos(lon)
        y = (n + h) * math.cos(lat) * math.sin(lon)
        z = (n * (1 - self.sqr_e) + h) * math.sin(lat)
        p = np.array([x, y, z, 0]).T
        return p


class rtk():
    def __init__(self, port = '/dev/ttyS5', bps = 115200, timeout=2):
        self.q = Queue(2)
        self.base_lon = 121.24554536
        self.base_lat = 30.87471633
        self.base_h = 27.427
        self.wgs84 = WGS84(self.base_lon, self.base_lat, self.base_h)
        self.__fix = 4 # rtk_fix mod value
        self.__ser = serial.Serial(port, bps, timeout=timeout)
        self.__qsize = 2
        self.__position = {
                'p':None,
                'angle': 0.0,
                'precision': 0.0,
                'angle_precision':0.0,
                'rtcm':'',
                'is_fix':False
                }
        self.__p = threading.Thread(target=self.__recv_rtk)
        self.__p_rtcm = threading.Thread(target=ntripClient.main_loop)
        self.__p.setDaemon(True)
        self.__p_rtcm.setDaemon(True)
        self.__p.start()
        self.__p_rtcm.start()

    def get_data(self):
        try:
            return self.q.get_nowait()
        except Exception as e:
            return None

    def __push_data(self, data):
        try:
            if self.q.full():
                self.q.get_nowait()
            # 转坐标
            p = data['p']
            p = p + self.wgs84.translation
            data['p'] =np.dot(p, self.wgs84.rotation)
            self.q.put_nowait(data)
        except Exception as e:
            print(e)

    def __recv_rtk(self):
        # rtk
        ser = self.__ser
        rtk_mod = 0
        while True:
            try:
                if ser.in_waiting:
                    #print('s')
                    data = ser.read_until('\n')
                    # print(data)
                    if 'GPGGA' in data:
                        data = data.split(',')
                        # print(data)
                        rtk_mod = int(data[6])
                        if rtk_mod != self.__fix:
                            self.__position['is_fix'] = False
                        else:
                            self.__position['is_fix'] = True
                        self.__position['is_fix'] = rtk_mod
                        self.__position['rtcm'] = data[13]
                        lon = float(data[4]) / 100.0
                        lat = float(data[2]) / 100.0
                        h = float(data[9])
                        # dddmm.mmmmm转度
                        lon = int(lon) + (lon - int(lon)) * 100 / 60.0
                        # ddmm.mmmmm转度
                        lat = int(lat) + (lat - int(lat)) * 100 / 60.0
                        self.__position['p'] = self.wgs84.calc_xyz(lon, lat, h)
                        self.__position['precision'] = float(data[8])
                        self.__push_data(self.__position)
                    elif 'HEADINGA' in data:
                        data = data.split(';')[1]
                        data = data.split(',')
                        self.__position['angle'] = float(data[3])
                        self.__position['precision_angle'] = float(data[6])
            except Exception as e:
                print(e)



def key_event():
    global enter_flag
    global space_flag
    global ctr_flag
    global left_enter_flag
    pygame.init()
    screen = pygame.display.set_mode((600,600))
    screen.fill((255,255,255))
    while True:
        '''
        keys = pygame.key.get_pressed()
        print(keys[pygame.K_RIGHT])
        time.sleep(0.5)
        '''
        events = pygame.event.get()
        time.sleep(0.2)
        for event in events:
            try:
                event_type = event.type
                event_key = event.dict['key']
                # print(event_key)
                if event_type == 2 and event_key == 13:
                    enter_flag = True
                elif event_type == 2 and event_key == 306:
                    ctr_flag = True
                elif event_type == 2 and event_key == 32:
                    space_flag = True
                elif event_type == 2 and event_key == 271:
                    left_enter_flag = True
            except:
                pass
    # keyboard.hook(press_keys)
    # keyboard.wait()


if __name__ == '__main__':
    rtk_rover = rtk()
    global enter_flag
    global space_flag
    global ctr_flag
    global left_enter_flag
    space_flag = False
    enter_flag = False
    ctr_flag = False
    left_enter_flag = False
    p = threading.Thread(target=key_event)
    p.setDaemon(True)
    p.start()
    position_list = []
    while True:
        data = rtk_rover.get_data()
        if not data:
            continue
        p = data['p']
        # p = p + rtk_rover.wgs84.translation
        # p =np.dot(p, rtk_rover.wgs84.rotation)
        # print('rtk: %d, x: %.3f, y: %.3f, z: %.3f, angle: %.2f'%(data['is_fix'], p[0], p[1], p[2], data['angle']))
        if space_flag:
            # 自动记录
            # space_flag = False
            # print('auto_recortd')
            try:
                pre_p = position_list[-1]
                dx = pre_p[0] - p[0]
                dy = pre_p[1] - p[1]
                len_p = dx**2 + dy**2
                print(len_p)
                if len_p >= 0.25 and data['is_fix'] == 4:
                    # position_list.append([p[0], p[1], p[2]])
                    print('aoto record: %d'%len(position_list))
                    print('rtk: %d, x: %.3f, y: %.3f, z: %.3f, angle: %.2f, rtcm:%s'%(data['is_fix'], p[0], p[1], p[2], data['angle'], data['rtcm']))
            except Exception as e:
                print(e)
        if left_enter_flag:
            # 关闭自动记录，手动记录
            space_flag = False
            left_enter_flag = False
            position_list.append([p[0], p[1], p[2]])
            print('record')
            print('rtk: %d, x: %.3f, y: %.3f, z: %.3f, angle: %.2f, rtcm:%s'%(data['is_fix'], p[0], p[1], p[2], data['angle'], data['rtcm']))
        if enter_flag:
            # 保存
            enter_flag = False
            temp = np.array(position_list)
            try:
                np.savetxt('position_list.txt', temp)
                print('saved')
            except Exception as e:
                print(e)
            print('rtk: %d, x: %.3f, y: %.3f, z: %.3f, angle: %.2f, rtcm:%s'%(data['is_fix'], p[0], p[1], p[2], data['angle'], data['rtcm']))
        if ctr_flag:
            # 刷新
            ctr_flag = False
            print('rtk: %d, x: %.3f, y: %.3f, z: %.3f, angle: %.2f, rtcm:%s'%(data['is_fix'], p[0], p[1], p[2], data['angle'], data['rtcm']))


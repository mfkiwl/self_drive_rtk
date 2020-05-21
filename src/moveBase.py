# _*_ coding: UTF-8 _*_

import sys
import json
import time
from math import cos, sin
import serialCmd
from motorTongyiIxL import Motor
# from motorDMK import Motor
from redisHandler import redisHandler


class moveBase(redisHandler):
    """
    机器人底盘
    """
    def init(self):
        self.pub_topics = ['move_base_out']
        self.sub_topics = ['move_base_in']
        self.left_wheel = Motor(2, '/dev/ttyS2')
        self.right_wheel = Motor(1, '/dev/ttyS3')
        self.base_info = {'left':[0,0,0,0], 'right':[0,0,0,0]}
        self.init_motor()
        self.start_sub()
    
    def init_motor(self):
        self.left_wheel.ini_motor()
        self.right_wheel.ini_motor()
    
    def run(self):
        # 主进程
        self.init()
        pre_time = time.time()
        timeout = 2.0
        while True:
            try:
                now = time.time()
                if now - pre_time >= timeout:
                    pre_time = now
                    print('time out')
                    self.write_speed(0, 0, 0)
                data= self.q_get_nowait()
                if not data:
                    time.sleep(0.1)
                    continue
                header = data['header']
                data = data['data']
                if header == 'init':
                    pre_time = time.time()
                    self.init_motor()
                elif header == 'speed':
                    pre_time = time.time()
                    self.write_speed(data['y'], data['angle'], data['speed'])
                elif header == 'heartbeat':
                    pre_time = time.time()
                    base_info = self.read_base_info()
                    data_pub = {
                            'header':'base_info',
                            'data':base_info
                            }
                    self.pub_all(data_pub)
            except Exception as e:
                print('move base err')
                print(e)

    def write_speed(self, y, angle, speed):
        """
        :param y:
        :param angle:
        :param speed
        :return:
        """
        # 根据车体样式更改，前后，左右
        # y = -y
        # angle = -angle
        l = 0.0
        r = 0.0
        if angle >= 0:
            l = 1
            r = - 1 + 2 * sin(angle)
        else:
            r = -1
            l = 1 + 2 * sin(angle)
        l  = l * speed / 100.0 * y
        r  = r * speed / 100.0 * y
        print('speed_l: ', l, 'speed_r: ', r)
        self.left_wheel.write_speed(l)
        self.right_wheel.write_speed(r)
        return True

    def read_speed(self):
        """
        :return: v, v_yaw, 单位： m/s, rad/s
        """
        v_l = self.left_wheel.read_speed()
        v_r = self.right_wheel.read_speed()
        return v_l, v_r

    def read_base_info(self):
        """
        :return: v, u, i, err_code 单位： r/min, V, A
        """
        info_l = self.left_wheel.read_base_info()
        info_r = self.right_wheel.read_base_info()
        return {'left': info_l, 'right': info_r}


if __name__ == '__main__':
    car = moveBase()
    car.run()

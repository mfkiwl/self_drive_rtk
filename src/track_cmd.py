# coding:utf-8
"""

Path tracking simulation with pure pursuit steering and PID speed control.

author: Atsushi Sakai (@Atsushi_twi)
        Guillaume Jacquenot (@Gjacquenot)

"""
import math
import time
import sys
import json
import requests
import redis
from redisHandler import redisHandler


class track_cmd(redisHandler):
    '''
    # tracking 路径跟踪
    '''
    def __init__(self):
        redisHandler.__init__(self)
        self.url = 'http://117.184.129.18:8000/planning/query/?key='
        self.sub_topics = ['rtk_out', 'track_cmd_in']
        self.pub_topics = ['ctrl_in']
        self.start_sub()

    def run(self):
        # 路径跟踪方法
        # 目标点误差
        delta_goal = 1.0
        data_cmd = {
                'header':'cmd',
                'data': ''
                }
        # 命令路径点表
        cmd_list=[]
        tracking_flag = False
        # 命令执行位置公差, 单位m
        while True:
            data = self.q_get_nowait()
            if data:
                header = data['header']
                data = data['data']
            else:
                header = None
            if header == 'rtk_position':
                # rtk 位置信息
                position = data
                if position['rtk_mod'] == 4 and tracking_flag:
                    # 自动时，发速度指令至电机
                    x = position['p'][0]
                    y = position['p'][1]
                    try:
                        for item in cmd_list:
                            x0 = item[0][0]
                            y0 = item[0][1]
                            cmd = item[1]
                            d = math.sqrt((x-x0)**2 + (y-y0)**2)
                            # print(d)
                            if d <= delta_goal:
                                for i_cmd in cmd:
                                    print(i_cmd)
                                    data_cmd['data'] = i_cmd
                                    self.pub_all(data_cmd)
                                    time.sleep(0.2)
                    except Exception as e:
                        print(e)
            elif header == 'auto_on':
                # 开启自动, 获取任务信息
                url = self.url + data
                response = requests.get(url)
                if response.status_code == 200:
                    try:
                        r_json = response.json()
                        cmd_list = r_json['cmd']
                        print(cmd_list)
                        tracking_flag = True
                    except Exception as e:
                        print(e)
                        tracking_flag = False

            elif header == 'auto_off':
                # 关闭自动
                tracking_flag = False


if __name__ == '__main__':
    print("Pure pursuit path tracking start")
    node_track = track_cmd()
    node_track.run()

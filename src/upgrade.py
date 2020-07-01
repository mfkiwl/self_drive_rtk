# coding:utf-8

import subprocess
import os
import sys


def run():
    # 运行脚本
    # dst = os.path.split(os.path.realpath(__file__))[0]
    dst = os.path.dirname(os.path.abspath(sys.argv[0]))
    print(dst)
    cmd = dst + '/start.sh'
    p = subprocess.Popen(cmd, shell=True,stdout=subprocess.PIPE, stderr=subprocess.STDOUT)

def start():
    upgrade()
    run()

def upgrade():
    path = get_path()
    if path:
        cp_file(path)

def cp_file(src):
    # dst = os.path.split(os.path.realpath(__file__))[0]
    dst = os.path.dirname(os.path.abspath(sys.argv[0]))
    print(dst)

    cmd = "cp -r %s %s" % (src, dst)
    print(cmd)
    status = subprocess.call(cmd, shell=True)
    if status != 0:
        # 更新失败，重启
        os.system('init 6')


def get_path():
    path = '/media/'
    root_dir = 'sweet_update'
    # 用户列表
    users = os.listdir(path)
    for username in users:
        tmp_user_path = path + username
        files = os.listdir(tmp_user_path)
        for filename in files:
            tmp_path = tmp_user_path + '/'  + filename
            files = os.listdir(tmp_path)
            if root_dir in files:
                path = tmp_path + '/' + root_dir + '/*'
                return path
    return None

start()

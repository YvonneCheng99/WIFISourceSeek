import cflib.crtp
import numpy as np
import pandas as pd
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.position_hl_commander import PositionHlCommander
import queue
import time
import random
import math
import Gaussian_progress as gp


# URI to the Crazyflie to connect to
height = 0.1  # 飞行高度，与搜寻的wifi源在同一高度
uri = 'radio://0/16/2M/E7E7E7E7E7'
threshold_rssi = -15  # 临界信号强度值，当rssi大于该值时认为找到源
random_distance = 0.3  # 刚开始随机方向飞行每次飞行的距离
random_times = 3  # 刚开始随机飞行的次数
distance = 0.2  # 向计算出的强度最大的位置移动的距离
scan_time = 10   # 无人机飞到某一位置悬停扫描的时间
coordinate = queue.Queue()  # 坐标队列
# coordinate_lifo = queue.LifoQueue()  # 后进先出的坐标队列
rssi_value = queue.Queue()  # 信号强度值队列，与坐标队列为一一对应的关系


# 计算方位角
def calculate_angle(x1, y1, x2, y2):
    angle = 0.0
    dx = x2 - x1
    dy = y2 - y1
    if dx == 0:
        if dy > 0:
            angle = 0.0
        elif dy < 0:
            angle = math.pi
    elif dy == 0:
        if dx > 0:
            angle = math.pi/2.0
        elif dy < 0:
            angle = math.pi * 3 / 2.0
    elif dx > 0:
        if dy > 0:
            angle = math.atan(dx / dy)
        elif dy < 0:
            angle = math.pi / 2 + math.atan(-dy / dx)
    elif dx < 0:
        if dy > 0:
            angle = 3.0 * math.pi / 2 + math.atan(dy / -dx)
        elif dy < 0:
            angle = math.pi + math.atan(dx / dy)
    return angle


def get_avg_rssi(scf):
    print(scf.rssi_list)
    scf.rssi_list.clear()
    while(len(scf.rssi_list) < 3):
        time.sleep(1)
    # time.sleep(scan_time)
    avg_rssi = sum(scf.rssi_list) / len(scf.rssi_list)
    return avg_rssi


def fin_max(ls):
    max_rssi = -100
    index = -1
    for i in range(0, len(ls)):
        if ls[i] > max_rssi:
            max_rssi = ls[i]
            index = i
    return index, max_rssi


def get_max_point():
    gpr = gp.GPR(optimize=True)
    test_d1 = np.arange(-1.5, 1.5, 0.1)
    test_d2 = np.arange(-1.5, 1.5, 0.1)
    x_d1 = np.arange(-1.5, 1.5, 0.1)
    y_d2 = np.arange(-1.5, 1.5, 0.1)
    test_d1, test_d2 = np.meshgrid(test_d1, test_d2)
    test_X = [[d1, d2] for d1, d2 in zip(test_d1.ravel(), test_d2.ravel())]
    total_sample = coordinate.qsize()
    coordinate_list = list(coordinate.queue)
    rssi_value_list = list(rssi_value.queue)

    for i in (0, total_sample):
        gpr.fit(coordinate_list, rssi_value_list)
        mu, cov = gpr.predict(test_X)
        max_index, max_mu = fin_max(mu)
        return test_X[max_index], max_mu


def seek_wifi_source():
    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        with PositionHlCommander(
                scf
                # x=0.0, y=0.0, z=0.0,
                # default_velocity=0.3,
                # default_height=height,
                # controller=PositionHlCommander.CONTROLLER_PID
                ) as pc:
            pc.go_to(0.0, 0.0, height)
            print("goto:0.0, 0.0")
            # Go to a coordinate
            index = 0  # 无人机移动次数
            flag = True  # 标志位，是否还要继续寻找
            last_coordinate = []  # 目前无人机所在位置的坐标
            while flag:
                if index < 3:
                    # 随机走random_distance米，走随机的角度
                    random_angel = random.uniform(0.0, math.pi * 2)
                    random_x = random_distance * math.sin(random_angel)
                    random_y = random_distance * math.cos(random_angel)
                    pc.go_to(random_x, random_y, height)
                    print("randomgoto:" + str(random_x) + "," + str(random_y))
                    # 获取信号强度数值
                    avg_rssi = get_avg_rssi(scf)
                    point_coordinate = [random_x, random_y, height]
                    point_coordinate_2d = [random_x, random_y]
                    coordinate.put(point_coordinate_2d)
                    last_coordinate = point_coordinate
                    # coordinate_lifo.put(point_coordinate)
                    rssi_value.put(avg_rssi)
                    df = pd.DataFrame({"coordinate": [point_coordinate_2d], "rssi": avg_rssi})
                    df.to_csv('test_1.csv', mode='a', index=None, header=None)
                    if avg_rssi > threshold_rssi:
                        flag = False
                        return point_coordinate_2d, avg_rssi
                    index += 1
                else:
                    # 调用高斯过程计算得到信号强度值最大的点的坐标
                    max_cordinate_predicted, max_rssi_predicted = get_max_point()
                    largest_x = max_cordinate_predicted[0]
                    largest_y = max_cordinate_predicted[1]
                    # 当前位置坐标
                    position_now = last_coordinate
                    angle = calculate_angle(position_now[0], position_now[1], largest_x, largest_y)
                    delta_x = distance * math.sin(angle)
                    detla_y = distance * math.cos(angle)
                    point_coordinate = [position_now[0]+delta_x, position_now[1]+detla_y, height]
                    point_coordinate_2d = [position_now[0]+delta_x, position_now[1]+detla_y]
                    pc.go_to(position_now[0]+delta_x, position_now[1]+detla_y, height)
                    print("disgoto:" + str(position_now[0]+delta_x) + "," + str(position_now[1]+detla_y))
                    # 获取信号强度数值
                    avg_rssi = get_avg_rssi(scf)
                    coordinate.put(point_coordinate_2d)
                    # coordinate_lifo.put(point_coordinate)
                    last_coordinate = point_coordinate
                    rssi_value.put(avg_rssi)
                    df = pd.DataFrame({"coordinate": [point_coordinate_2d], "rssi": avg_rssi})
                    df.to_csv('test_1.csv', mode='a', index=None, header=None)
                    if avg_rssi > threshold_rssi:
                        flag = False
                        return point_coordinate_2d, avg_rssi
                    index += 1


if __name__ == '__main__':
    cflib.crtp.init_drivers()
    seek_wifi_source()

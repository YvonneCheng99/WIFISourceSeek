import source_seek as ss
import numpy as np
import queue
import pandas as pd
import Gaussian_progress as gp


coordinate = queue.Queue()  # 坐标队列
rssi_value = queue.Queue()  # 信号强度值队列，与坐标队列为一一对应的关系

def init_queue():
    data = pd.read_csv("test.csv", header = None)
    # print(data)
    data_1 = data.iloc[:,0]
    coordinate_list = np.array(data_1)
    for a in coordinate_list:
        str_list = a.split('[')[1]
        str_list = str_list.split(']')[0]
        coor = str_list.split(',')
        coordinate.put([float(coor[0]), float(coor[1])])

    data_2 = data.iloc[:,1]
    rssi_list = np.array(data_2)
    for a in rssi_list:
        rssi_value.put(float(a))


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


def main():
    init_queue()
    result_coordinate, max_rssi = get_max_point()
    print(result_coordinate)
    print(max_rssi)


if __name__ == '__main__':
    main()
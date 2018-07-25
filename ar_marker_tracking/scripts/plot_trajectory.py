#!/usr/bin/env python
# coding: UTF-8

import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D


def getList(path):
    file = open(path, 'r')

    lines = file.readlines() 

    trajectory = list()
    
    length = len(lines)
    for i in range(1, length-1):
        line_l = lines[i].split(',')

        l = [0]*7
        for i in range(6):
            l[i] = float(line_l[i])
            print l
        trajectory.append(l)

    return trajectory


def plotTrajectory(trajectory):

    # グラフ作成
    fig = plt.figure()
    ax = Axes3D(fig)
    # ax = fig.add_subplot(111, projection='3d')

    # 軸ラベルの設定
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_zlabel("z [m]")

    # # 表示範囲の設定
    # ax.set_xlim(0, 0.5)
    # ax.set_ylim(-0.25, 0.25)
    # ax.set_zlim(0, 0.2)

    # 抽出条件設定
    
    
    # d1 = d[d[:,0] >= 7]
    # d2 = d[(d[:,0] < 7) & ((d[:,1] > 3) & (d[:,1] <= 3.5))]
    # d3 = d[(d[:,0] < 7) & ((d[:,1] <= 3) | (d[:,1] > 3.5))]

    list_x = []
    list_y = []
    list_z = []
    for value in trajectory:
        list_x.append(value[0])
        list_y.append(value[1])
        list_z.append(value[2])

    # グラフ描画
    ax.plot(list_x, list_y, list_z, "o", color="#cccccc", ms=4, mew=0.5, label='data')
    # ax.scatter(list_x, list_y, list_z, marker="o", c="#ff0000", s=10, label="data")

    # ax.scatter([0], [0], [0], marker='o', c='#00cccc', s=10 , label='world')

    # ax.scatter([list_x[0]], [list_y[0]], [list_z[0]], marker="o", c="#ff0000", s=10, label='Start')
    # ax.scatter([list_x[-1]], [list_y[-1]], [list_z[-1]], marker="o", c="yellow", s=10, label='Goal')

    ax.plot([0], [0], [0], 'o', color='#00cccc', ms=6 , label='/world')

    ax.plot([list_x[0]], [list_y[0]], [list_z[0]], "o", color="#ff0000", ms=6, label='Start')
    ax.plot([list_x[-1]], [list_y[-1]], [list_z[-1]], "o", color="yellow", ms=6, label='Goal')

    ax.legend()
    # ax.plot(d1[:,0], d1[:,1], d1[:,2], "o", color="#cccccc", ms=4, mew=0.5)
    # ax.plot(d2[:,0], d2[:,1], d2[:,2], "o", color="#00cccc", ms=4, mew=0.5)
    # ax.plot(d3[:,0], d3[:,1], d3[:,2], "o", color="#ff0000", ms=4, mew=0.5)
    plt.show()


if __name__ == '__main__':

    file_path = '../data/trajectory.txt'

    trajectory = getList(file_path)
    plotTrajectory(trajectory)
    
    #     print (line.strip("\n"))
    # print (line.rstrip("\n"))
    # print (f.read())
#     lines = f.readlines()

# line_strip = [line.strip() for line in lines]

# for line in 

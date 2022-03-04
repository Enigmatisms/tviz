"""
    轨迹绘制以及 evaluation
"""
from crypt import methods
import os
import numpy as np
import matplotlib.pyplot as plt
from sys import argv
from evals import chamferDistanceEval

colors = ((1.0, 0, 0), (0, 0.5, 0), (0, 0, 1.0), (0.3, 0.3, 0.3))
# methods = ("Chain SLAM ", "GMapping ", "Cartographer ", "GT ")
methods = []
attributes = ("X axis", "Y axis", "Rotation")

# 时间戳并不能完全对应

def readFromFile(path:str)->np.array:
    with open(path, 'r') as file:
        raw_all = file.readlines()
        result = np.zeros((len(raw_all), 4))
        for i, line in enumerate(raw_all):
            stripped = line.split(' ')
            result[i, 0] = float(stripped[0]) / 1e9
            for j in range(1, 3):
                result[i, j] = float(stripped[j])
            angle = float(stripped[-1][:-1])
            while angle > np.pi:
                angle -= np.pi * 2
            while angle < -np.pi:
                angle += np.pi * 2
            result[i, -1] = angle
        return result

# 按照二维位置绘制
def plotTrajectory(data:tuple, show = True):
    method_num = len(data)
    for i in range(method_num):
        plt.plot(data[i][:, 1], data[i][:, 2], color = colors[i], label = methods[i] + 'trajectory')
        plt.scatter(data[i][:, 1], data[i][:, 2], color = colors[i], s = 6)
    plt.title('Top down orthogonal trajectories')
    plt.grid(axis = 'both')
    plt.legend()
    if not show: return
    plt.show()
    plt.show()

# 按照时间戳以及维度绘制
def plotTrajDimWise(all_data:tuple, show = True):
    method_num = len(all_data)
    for i in range(3):
        plt.subplot(3, 1, i + 1)
        for j in range(method_num):
            plt.plot(all_data[j][:, 0], all_data[j][:, i + 1], color = colors[j], label = methods[j] + attributes[i])
        plt.grid(axis = 'both')
        plt.legend()
        plt.title('Dimension-wise trajectories')
    if not show: return
    plt.show()

def gtChamferDist(pts1, pts2):
    length = len(pts1)
    err = 0
    for i in range(length):
        diff:np.ndarray = pts1[i, 1:] - pts2[i, 1:]
        err += diff.dot(diff)
    err /= length
    return err ** 0.5

def showTrajectories(map_name):
    cslam_traj = readFromFile(map_name + "c_traj.txt")
    cslam_gt = readFromFile(map_name + "c_gt.txt")
    print("CSLAM trajectory evaluation:")
    for dim in range(3):
        print("Dimension %s error: %f"%(attributes[dim], chamferDistanceEval(cslam_traj, cslam_gt, dim + 1)))
    print("Point wise chamfer distance: %f"%(chamferDistanceEval(cslam_traj, cslam_gt, -1)))
    print(" ===================================================")
    all_data = [cslam_traj]
    methods.append("Chain SLAM ")
    if os.path.exists(map_name + "gmap_traj.txt"):
        gmap_traj = readFromFile(map_name + "gmap_traj.txt")
        print("GMapping trajectory evaluation:")
        for dim in range(3):
            print("Dimension %s error: %f"%(attributes[dim], chamferDistanceEval(gmap_traj, cslam_gt, dim + 1)))
        print("Point wise chamfer distance: %f"%(chamferDistanceEval(gmap_traj, cslam_gt, -1)))
        methods.append("GMapping ")
        all_data.append(gmap_traj)
        print(" =================================================== ")
    if os.path.exists(map_name + "carto_traj_0.txt"):
        carto_traj = readFromFile(map_name + "carto_traj_0.txt")
        print("Cartographer trajectory evaluation:")
        for dim in range(3):
            print("Dimension %s error: %f"%(attributes[dim], chamferDistanceEval(carto_traj, cslam_gt, dim + 1)))
        print("Point wise chamfer distance: %f"%(chamferDistanceEval(carto_traj, cslam_gt, -1)))
        methods.append("Cartographer ")
        all_data.append(carto_traj)
        print(" =================================================== ")

    methods.append("GT ")
    all_data.append(cslam_gt)

    # all_data = (gmap_traj, cslam_gt, cslam_traj)
    # print("GMapping trajectory evaluation:")
    # for dim in range(3):
    #     print("Dimension %s error: %f"%(attributes[dim], chamferDistanceEval(gmap_traj, gmap_gt, dim + 1)))
    plt.figure(0)
    plotTrajDimWise(all_data, False)
    plt.figure(1)
    plotTrajectory(all_data)

if __name__ == "__main__":
    if len(argv) < 2:
        print("Usage: python ./plot_traj.py <map folder name>")
        exit(-1)
    map_name = argv[1]
    showTrajectories(map_name)
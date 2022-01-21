"""
    evaluation
"""
from typing import Any
import numpy as np
from numpy.linalg import norm

def binarySearch(query, data)->int:
    s, e = 0, len(data), 
    m = (s + e) >> 1
    while s < e:
        val = data[m]
        if val > query:
            e = m
        elif val < query:
            s = m + 1
        else: return m
        m = (s + e) >> 1
    return m

def linearInterp(x1, x2_1, x2_2, y1, y2)->float:
    if abs(x2_1 - x2_2) < 1e-7:
        return y1
    k = (x1 - x2_1) / (x2_2 - x2_1)
    return y1 * (1 - k) + y2 * k

def singleValueEval(query:float, dim:int, key_num:int, all_keys:list, traj_eval:np.ndarray = None)->Any:
    k_id = binarySearch(query, all_keys[:, 0])
    if k_id >= key_num or k_id == 0:
        k_id = min(key_num - 1, k_id)
        if not traj_eval is None:
            return all_keys[k_id, 1:]
        st = all_keys[k_id, 0]
        return linearInterp(st, st, st, all_keys[k_id, dim], all_keys[k_id, dim])
    else:
        if not traj_eval is None:
            p1, p2 = all_keys[k_id - 1, 1:-1], all_keys[k_id, 1:-1]
            if norm(traj_eval - p1) <= norm(traj_eval - p2):
                return all_keys[k_id - 1, 1:]
            return all_keys[k_id, 1:]
        st = all_keys[k_id - 1, 0]
        st2 = all_keys[k_id, 0]      # 时间戳
        return linearInterp(query, st, st2, all_keys[k_id - 1, dim], all_keys[k_id, dim])

def chamferDistanceEval(data:np.ndarray, gt:np.ndarray, dim:int = 0, sqr:bool = False, single_side:bool = False)->float:
    """
        #### Chamfer 距离：基于时间戳二分查找的轨迹 或 单一维度误差metrics计算
        ---
        - data: SLAM轨迹
        - gt: 真实轨迹
        - dim: 在单一维度误差计算中，指定需要计算的维度, dim < 0 时计算轨迹误差
        - sqr: 是否使用 mean squared root error
        - single_side: 是否只计算单边误差（data->gt）
    """
    d1, d2 = len(data), len(gt)
    err = 0
    for i in range(d1):
        query = data[i, 0]
        if dim < 0:
            p = singleValueEval(query, dim, d2, gt, data[i, 1:-1])
            diff:np.ndarray = p - data[i, 1:]
            err += diff.dot(diff)
        else:
            value = singleValueEval(query, dim, d2, gt)
            err += (data[i, dim] - value) ** 2 if sqr else abs(data[i, dim] - value)
    if single_side == False:
        for i in range(d2):
            query = gt[i, 0]
            if dim < 0:
                p = singleValueEval(query, dim, d1, data, gt[i, 1:-1])
                diff:np.ndarray = p - gt[i, 1:]
                err += diff.dot(diff)
            else:
                value = singleValueEval(query, dim, d1, data)
                err += (gt[i, dim] - value) ** 2 if sqr else abs(gt[i, dim] - value)
        err /= (d1 + d2)
    else:
        err /= d1
    if sqr or (dim < 0):
        return err ** 0.5
    return err

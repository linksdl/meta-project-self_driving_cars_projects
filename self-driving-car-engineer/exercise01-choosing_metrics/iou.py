"""
# !/usr/bin/env python
# -*- coding: utf-8 -*-
@Time        : 2022/2/23 16:43
@Author      : shengdl999links@gmail.com
@ProjectName : self-driving-car-engineer
@File        : iou.py
"""

import numpy as np
from utils import get_data, check_results


def calculate_iou(gt_bbox, pred_bbox):
    """
    calculate iou
    args:
    - gt_bbox [array]: 1x4 single gt bbox
    - pred_bbox [array]: 1x4 single pred bbox
    returns:
    - iou [float]: iou between 2 bboxes
    参考： https://blog.csdn.net/briblue/article/details/91366128
    """
    x_min = np.max([gt_bbox[0], pred_bbox[0]])
    y_min = np.max([gt_bbox[1], pred_bbox[1]])
    x_max = np.min([gt_bbox[2], pred_bbox[2]])
    y_max = np.min([gt_bbox[3], pred_bbox[3]])

    # calculate the intersection between two area
    # 两个区域面积的交集
    intersection = max(0, x_max - x_min) * max(0, y_max - y_min)

    # calculate the area
    gt_area = (gt_bbox[2] - gt_bbox[0]) * (gt_bbox[3] - gt_bbox[1])
    # calculate the predication area
    pred_area = (pred_bbox[2] - pred_bbox[0]) * (pred_bbox[3] - pred_bbox[1])

    # 两个区域面积的并集
    union_area = gt_area + pred_area - intersection
    # calculate the iou score [0, 1]
    iou = intersection / union_area
    return iou


def calculate_ious(gt_bboxes, pred_bboxes):
    """
    calculate ious between 2 sets of bboxes
    args:
    - gt_bboxes [array]: Nx4 ground truth array
    - pred_bboxes [array]: Mx4 pred array
    returns:
    - iou [array]: NxM array of ious
    """

    ious = np.zeros((gt_bboxes.shape[0], pred_bboxes.shape[0]))
    for i, gt_bbox in enumerate(gt_bboxes):
        for j, pred_bbox in enumerate(pred_bboxes):
            ious[i, j] = calculate_iou(gt_bbox, pred_bbox)
    return ious

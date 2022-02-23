"""
# !/usr/bin/env python
# -*- coding: utf-8 -*-
@Time        : 2022/2/23 17:21
@Author      : shengdl999links@gmail.com
@ProjectName : self-driving-car-engineer
@File        : precison_recall.py
"""


import numpy as np
from iou import calculate_ious
from utils import get_data


def precision_recall(ious, gt_classes, pred_classes):
    """
    calculate precision and recall
    args:
    - ious [array]: NxM array of ious
    - gt_classes [array]: 1xN array of ground truth classes
    - pred_classes [array]: 1xM array of pred classes
    returns:
    - precision [float]
    - recall [float]
    博客：https://blog.csdn.net/HunGRy_FOOliSHhh/article/details/112385606?spm=1001.2101.3001.6650.15&utm_medium=distribute.pc_relevant.none-task-blog-2%7Edefault%7EBlogCommendFromBaidu%7ERate-15.pc_relevant_default&depth_1-utm_source=distribute.pc_relevant.none-task-blog-2%7Edefault%7EBlogCommendFromBaidu%7ERate-15.pc_relevant_default&utm_relevant_index=22
    """

    xs, ys = np.where(ious > 0.5)

    # calculate the true positive(TP) and ture negative(TN)
    tps = 0
    fps = 0

    for x, y in zip(xs, ys):
        if gt_classes[x] == pred_classes[y]:
            tps += 1
        else:
            fps += 1

    matched_gt = len(np.unique(xs))
    # False Negative
    fns = len(gt_classes) - matched_gt

    # precision = TP / (TP + FP)
    precision = tps / (tps + fps)
    # recall  = TP / (TP + FN)
    recall = tps / (tps + fns)

    return precision, recall


if __name__ == "__main__":
    ground_truth, predictions = get_data()

    # get bboxes array
    filename = 'segment-1231623110026745648_480_000_500_000_with_camera_labels_38.png'
    gt_bboxes = [g['boxes'] for g in ground_truth if g['filename'] == filename][0]
    gt_bboxes = np.array(gt_bboxes)
    gt_classes = [g['classes'] for g in ground_truth if g['filename'] == filename][0]

    pred_bboxes = [p['boxes'] for p in predictions if p['filename'] == filename][0]
    pred_boxes = np.array(pred_bboxes)
    pred_classes = [p['classes'] for p in predictions if p['filename'] == filename][0]

    ious = calculate_ious(gt_bboxes, pred_boxes)
    precision, recall = precision_recall(ious, gt_classes, pred_classes)

    print(f'Precision: {precision}')
    print(f'Recall: {recall}')

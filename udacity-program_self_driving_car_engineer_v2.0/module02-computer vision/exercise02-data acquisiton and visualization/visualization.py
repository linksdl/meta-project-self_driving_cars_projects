"""
# !/usr/bin/env python
# -*- coding: utf-8 -*-
@Time        : 2022/2/23 19:35
@Author      : shengdl999links@gmail.com
@ProjectName : udacity-program_self_driving_car_engineer_v1.0_source.0
@File        : visualization.py
"""
import glob
import os.path
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from PIL import Image
from utils import get_data


def viz(ground_truth):
    """
    create a grid visualization of images with color coded bboxes
    args:
    - ground_truth [list[dict]]: ground truth data
    """
    # IMPLEMENT THIS FUNCTION
    paths = glob.glob('../data/images/*')

    gt_dic = {}

    # mapping to access data faster
    for gt in ground_truth:
        gt_dic[gt['filename']] = gt

    # color mapping of classes
    color_map = {1: [1, 0, 0], 2: [0, 1, 0], 4: [0, 0, 1]}

    f, ax = plt.subplots(4, 5, figsize=(20, 10))
    for i in range(20):
        x = i % 4
        y = i % 5

        filename = os.path.basename(paths[i])
        img = Image.open(paths[i])
        ax[x, y].imshow(img)

        bboxes = gt_dic[filename]['boxes']
        classes = gt_dic[filename]['classes']

        for cl, bb in zip(classes, bboxes):
            y1, x1, y2, x2 = bb
            rec = Rectangle((x1, y1), x2 - x1, y2 - y1, facecolor='none', edgecolor=color_map[cl])
            ax[x, y].add_patch(rec)
        ax[x, y].axis('off')

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    ground_truth, _ = get_data()
    viz(ground_truth)

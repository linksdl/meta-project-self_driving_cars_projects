#!/usr/bin/env python
# -*- coding: utf-8 -*-

# @Time        : 2022/2/23 15:59
# @Author      : linksdl
# @ProjectName : udacity-program_self_driving_car_engineer_v1.0_source.0
# @File        : utils.py

import json
import numpy as np


def check_results(ious):
    """
    check the results
    """
    solution = np.load('data/exercise1_check.npy')
    assert (ious == solution).sum() == 40, 'The ios calculation is wrong!'
    print('Congrats, the iou calculation is correct!')


def get_data():
    """
    get data
    """
    with open('data/ground_truth.json') as f:
        ground_truth = json.load(f)

    with open('data/predictions.json') as f:
        predictions = json.load(f)

    return ground_truth, predictions

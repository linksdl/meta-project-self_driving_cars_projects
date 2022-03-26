"""
# !/usr/bin/env python
# -*- coding: utf-8 -*-
@Time        : 2022/3/26 11:56
@File        : softmax.py
"""

import numpy as np


# Write a function that takes as input a list of numbers, and returns
# the list of values given by the softmax function.
def softmax(L):
    """
    softmax
    """
    exPL = np.exp(L)
    sumExpL = sum(exPL)

    result = []
    for i in exPL:
        result.append(i * 1.0/sumExpL)
    return result


# Note: The function np.divide can also be used here, as follows:
# def softmax(L):
#     expL = np.exp(L)
#     return np.divide (expL, expL.sum())


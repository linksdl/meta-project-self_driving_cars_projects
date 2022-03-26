"""
# !/usr/bin/env python
# -*- coding: utf-8 -*-
@Time        : 2022/3/26 18:13
@File        : softmax.py
"""

# Solution is available in the other "solution.py" tab
import numpy as np


def softmax(x):
    """Compute softmax values for each sets of scores in x."""
    # TODO: Compute and return softmax(x)

    return np.exp(x) / np.sum(np.exp(x), axis=0)
    # x = np.divide(np.exp(x), np.sum(np.exp(x)))
    # return x


logits = [3.0, 1.0, 0.2]
print(softmax(logits))
# [0.8360188  0.11314284 0.05083836]
# [0.8360188  0.11314284 0.05083836]

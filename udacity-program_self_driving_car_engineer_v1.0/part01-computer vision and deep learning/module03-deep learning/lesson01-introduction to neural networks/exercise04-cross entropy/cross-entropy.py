"""
# !/usr/bin/env python
# -*- coding: utf-8 -*-
@Time        : 2022/3/26 13:25
@File        : cross-entropy.py
"""

import numpy as np


# Write a function that takes as input two lists Y, P,
# and returns the float corresponding to their cross-entropy.
def cross_entropy(Y, P):
    """
    cross_entropy
    """
    Y = np.float_(Y)
    P = np.float_(P)

    return -np.sum(Y * np.log(P) + (1 - Y) * np.log(1 - P))

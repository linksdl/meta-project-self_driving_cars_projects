"""
# !/usr/bin/env python
# -*- coding: utf-8 -*-
@Time        : 2022/3/26 16:48
@File        : gd.py
"""


def gradient_descent_update(x, gradx, learning_rate):
    """
    Performs a gradient descent update.
    """
    # TODO: Implement gradient descent.

    # Return the new value for x
    delta_x = gradx * learning_rate
    x = x - delta_x
    return x

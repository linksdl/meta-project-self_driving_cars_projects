"""
# !/usr/bin/env python
# -*- coding: utf-8 -*-
@Time        : 2022/3/16 13:35
@Author      : shengdl999links@gmail.com
@ProjectName : udacity-program_sensor_fusion_v1
@File        : gaussian_function.py
"""

from math import *


def gaussion_f(mu, sigma2, x):
    """
    Gaussian function
    :param mu:
    :param sigma2:
    :param x:
    :return:
    """
    return 1 / sqrt(2 * pi * sigma2) * exp(-0.5 * (x - mu) / sigma2)


value = gaussion_f(10, 4, 10)
print(value)


# Write a program to update your mean and variance
# when given the mean and variance of your belief
# and the mean and variance of your measurement.
# This program will update the parameters of your
# belief function.
def update(mean1, var1, mean2, var2):
    """
    新的均值和方差 更新
    :param mean1:
    :param var1:
    :param mean2:
    :param var2:
    :return:
    """
    new_mean = (mean1 * var2 + mean2 * var1) / (var1 + var2)
    new_var = 1 / (1 / var1 + 1 / var2)
    return [new_mean, new_var]


print(update(10., 8., 13., 2.))


# Write a program that will predict your new mean
# and variance given the mean and variance of your
# prior belief and the mean and variance of your
# motion.

def update(mean1, var1, mean2, var2):
    """
    运动更新
    :param mean1:
    :param var1:
    :param mean2:
    :param var2:
    :return:
    """
    new_mean = (var2 * mean1 + var1 * mean2) / (var1 + var2)
    new_var = 1 / (1 / var1 + 1 / var2)
    return [new_mean, new_var]


def predict(mean1, var1, mean2, var2):
    """
    运动估计
    :param mean1:
    :param var1:
    :param mean2:
    :param var2:
    :return:
    """
    new_mean = mean1 + mean2
    new_var = var1 + var2
    return [new_mean, new_var]


print(predict(10., 4., 12., 4.))

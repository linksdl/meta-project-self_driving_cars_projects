"""
# !/usr/bin/env python
# -*- coding: utf-8 -*-
@Time        : 2022/3/17 23:21
@File        : predict_function.py
"""


# Write a program that will predict your new mean
# and variance given the mean and variance of your
# prior belief and the mean and variance of your
# motion.

def update(mean1, var1, mean2, var2):
    new_mean = (var2 * mean1 + var1 * mean2) / (var1 + var2)
    new_var = 1 / (1 / var1 + 1 / var2)
    return [new_mean, new_var]


def predict(mean1, var1, mean2, var2):
    new_mean = mean1 + mean2  # TODO: Update this!
    new_var = var1 + var2  # TODO: Update this!
    return [new_mean, new_var]


print(predict(10., 4., 12., 4.))

# Run this cell to test your code when you are satisfied with your results!
# import tester
#
# tester.test(predict)

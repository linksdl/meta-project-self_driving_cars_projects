"""
# !/usr/bin/env python
# -*- coding: utf-8 -*-
@Time        : 2022/3/17 23:09
@File        : new_mean_and_variance.py
"""


# Write a program to update your mean and variance
# when given the mean and variance of your belief
# and the mean and variance of your measurement.
# This program will update the parameters of your
# belief function.

def update(mean1, var1, mean2, var2):
    new_mean = (mean1 * var2 + mean2 * var1) / (var1 + var2)  # TODO: Update this!
    new_var = 1 / (1 /var1 + 1/ var2)   # TODO: Update this!
    return [new_mean, new_var]


print(update(10., 8., 13., 2.))


# Run this cell to test your code when you are satisfied with your results!
# import tester

# tester.test(update)

"""
# !/usr/bin/env python
# -*- coding: utf-8 -*-
@Time        : 2022/3/17 22:26
@File        : maximize_gaussian.py
"""

# For this problem, you aren't writing any code.
# Instead, please just change the last argument
# in f() to maximize the output.

from math import *


def f(mu, sigma2, x):
    return 1 / sqrt(2. * pi * sigma2) * exp(-.5 * (x - mu) ** 2 / sigma2)


print(f(10., 4., 8.))  # Change the 8. to something else!
# 0.12098536225957168
print(f(10., 4., 10.))  # Change the 10. to get Guassian Max value.
# 0.19947114020071635

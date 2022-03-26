"""
# !/usr/bin/env python
# -*- coding: utf-8 -*-
@Time        : 2022/3/26 18:24
@File        : numerical_stability.py
"""

a = 1000000000
for i in range(1000000):
    a = a + 1e-6
print(a - 1000000000)

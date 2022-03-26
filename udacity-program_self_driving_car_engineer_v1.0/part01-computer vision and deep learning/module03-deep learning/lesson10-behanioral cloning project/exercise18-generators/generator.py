"""
# !/usr/bin/env python
# -*- coding: utf-8 -*-
@Time        : 2022/3/26 23:54
@File        : generator.py
"""


def fibonacci():
    numbers_list = []
    while 1:
        if (len(numbers_list) < 2):
            numbers_list.append(1)
        else:
            numbers_list.append(numbers_list[-1] + numbers_list[-2])
        yield 1  # change this line so it yields its list instead of 1


our_generator = fibonacci()
my_output = []

for i in range(10):
    my_output = (next(our_generator))

print(my_output)

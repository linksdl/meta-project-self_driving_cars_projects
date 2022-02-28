"""
# !/usr/bin/env python
# -*- coding: utf-8 -*-
@Time        : 2022/2/24 22:24
@Author      : shengdl999links@gmail.com
@ProjectName : self-driving-car-engineer
@File        : perspective_transform.py
"""

import cv2

# 如何选择 source points
# many perspective transform algorithms will programmatically detect
# four source points in an image based on edge or corner detection
# and analyzing attributes like color and surrounding pixels.

# Compute the perspective transform, M, given source and destination points:
# M = cv2.getPerspectiveTransform(src, dst)

# Compute the inverse perspective transform
# Minv = cv2.getPerspectiveTransform(dst, src)


# Warp an image using the perspective transform, M:
# warped = cv2.warpPerspective(img, M, img_size, flags=cv2.INTER_LINEAR)





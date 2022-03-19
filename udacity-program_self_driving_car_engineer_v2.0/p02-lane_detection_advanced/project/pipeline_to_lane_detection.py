"""
# !/usr/bin/env python
# -*- coding: utf-8 -*-
@Time        : 2022/2/25 16:20
@Author      : shengdl999links@gmail.com
@ProjectName : udacity-program_self_driving_car_engineer_v1.0_source.0
@File        : pipeline_to_lane_detection.py
"""

# 1, Camera calibration
# 2, Distortion Correction for image
# 3, Implement a color and gradient threshold
# 4, Warp the image using perspective transform
# various combinations of color and gradient thresholds to generate a binary image

# ** Line Finding Method: Peaks in a Histogram**
#
# After applying calibration, thresholding,
# and a perspective transform to a road image,
# you should have a binary image where the lane lines stand out clearly.
# However, you still need to decide explicitly which pixels are part of the lines
# and which belong to the left line and which belong to the right line.

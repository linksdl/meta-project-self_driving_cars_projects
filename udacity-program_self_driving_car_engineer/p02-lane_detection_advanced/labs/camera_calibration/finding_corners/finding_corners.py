"""
# !/usr/bin/env python
# -*- coding: utf-8 -*-
@Time        : 2022/2/24 21:36
@Author      : shengdl999links@gmail.com
@ProjectName : udacity-program_self_driving_car_engineer
@File        : finding_corners.py
"""


import numpy as np
import cv2
import matplotlib.pyplot as plt
import matplotlib.image as mpimg

# prepare object points
nx = 0 # enter the number of inside corners in x
ny = 0 # enter the number of inside corners in y

# make a list of calibration images
calibration_file_name = 'calibration_test.png'
img = cv2.imread(calibration_file_name)

# convert to grayscale
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

# Find the chessboard corners
# Size patternsize(8,6); //interior number of corners
# 方法说明：https://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html#cv2.findChessboardCorners
nx = 8
ny = 6
ret, corners = cv2.findChessboardCorners(gray, (nx, ny), None)

# If found the pattern, draw corners
if ret is True:
    # draw and display the corners
    # https://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html#cv2.drawChessboardCorners
    cv2.drawChessboardCorners(img, (nx, ny), corners, ret)
    plt.imshow(img)

plt.show()





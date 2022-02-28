"""
# !/usr/bin/env python
# -*- coding: utf-8 -*-
@Time        : 2022/2/24 16:31
@Author      : shengdl999links@gmail.com
@ProjectName : self-driving-car-engineer
@File        : canny_edge_detection.py
"""


import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import cv2  #bringing in OpenCV libraries


image = mpimg.imread('exit-ramp.jpeg')
# plt.imshow(image)
# plt.show()

# grayscale conversion
gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
# plt.imshow(gray, cmap='gray')
# plt.show()

# Define a kernel size for Gaussian smoothing / blurring
# Note: this step is optional as cv2.Canny() applies a 5x5 Gaussian internally
kernel_size = 3
blur_gray = cv2.GaussianBlur(gray,(kernel_size, kernel_size), 0)

# Define parameters for Canny and run it
# NOTE: if you try running this code you might want to change these!
low_threshold = 1
high_threshold = 10
edges = cv2.Canny(blur_gray, low_threshold, high_threshold)

# Display the image
plt.imshow(edges, cmap='Greys_r')
plt.show()

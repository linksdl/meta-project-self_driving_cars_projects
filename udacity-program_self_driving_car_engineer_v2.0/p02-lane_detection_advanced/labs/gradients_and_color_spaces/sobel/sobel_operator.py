"""
# !/usr/bin/env python
# -*- coding: utf-8 -*-
@Time        : 2022/2/28 20:27
@Author      : shengdl999links@gmail.com
@ProjectName : udacity-program_self_driving_car_engineer_v2.0
@File        : sobel_operator.py
"""

import cv2
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg

img = mpimg.imread('curved-lane.jpeg')
# Use cv2.COLOR_RGB2GRAY if you've read in an image using mpimg.imread().
# Use cv2.COLOR_BGR2GRAY if you've read in an image using cv2.imread().
gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)

# Calculate the derivative in the xx direction (the 1, 0 at the end denotes xx direction):
sobelx = cv2.Sobel(gray, cv2.CV_64F, 1, 0)

# Calculate the derivative in the yy direction (the 0, 1 at the end denotes yy direction):
sobely = cv2.Sobel(gray, cv2.CV_64F, 0, 1)

# Calculate the absolute value of the xx derivative:
abs_sobelx = np.absolute(sobelx)
abs_sobely = np.absolute(sobely)

# Convert the absolute value image to 8-bit:
scaled_sobel = np.uint8(255 * abs_sobelx/np.max(abs_sobelx))
scaled_sobel = np.uint8(255 * abs_sobely/np.max(abs_sobely))

thresh_min = 20
thresh_max = 100
sxbinary = np.zeros_like(scaled_sobel)
sxbinary[(scaled_sobel >= thresh_min) & (scaled_sobel <= thresh_max)] = 1
plt.imshow(sxbinary, cmap='gray')
plt.show()



def abs_sobel_thresh(img, orient='x', thresh_min=0, thresh_max=255):
    # Grayscale
    # Apply cv2.Sobel()
    # Take the absolute value of the output from cv2.Sobel()
    # Scale the result to an 8-bit range (0-255)
    # Apply lower and upper thresholds
    # Create binary_output
    # return binary_output
    pass

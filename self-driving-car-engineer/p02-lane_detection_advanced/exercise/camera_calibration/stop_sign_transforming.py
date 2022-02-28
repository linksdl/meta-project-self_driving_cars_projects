"""
# !/usr/bin/env python
# -*- coding: utf-8 -*-
@Time        : 2022/2/24 22:41
@Author      : shengdl999links@gmail.com
@ProjectName : self-driving-car-engineer
@File        : stop_sign_transforming.py
"""

import numpy as np
import cv2
import matplotlib.pyplot as plt
import matplotlib.image as mpimg

# Read and display the original image
img = mpimg.imread('')

plt.imshow(img)
plt.show()


# source image points
plt.imshow(img)
plt.plot(850, 320, '.') # top right
plt.plot(865, 450, '.') # bottom right
plt.plot(533, 350, '.') # bottom left
plt.plot(535, 210, '.') # top left


def warp(img):
    # Define calibration box in source (original) and destination (desired or warped) coordinates
    img_size = (img.shape[1], img.shape[0])

    # Four source coordinates
    src = np.float32([
        [850, 320],
        [865, 450],
        [533, 350],
        [535, 210]
    ])

    # Four desired coordinates
    dst = np.float32([
        [870, 240],
        [870, 370],
        [520, 370],
        [520, 240]
    ])

    # Compute the perspective transform, M, given source and destination points:
    M = cv2.getPerspectiveTransform(src, dst)

    # Compute the inverse perspective transform
    Minv = cv2.getPerspectiveTransform(dst, src)

    # Warp an image using the perspective transform, M:
    warped = cv2.warpPerspective(img, M, img_size, flags=cv2.INTER_LINEAR)

    return warped


# get perspective transform
warped_im = warp(img)

# Viz undistortion

f, (ax1, ax2) = plt.subplots(1, 2, figsize=(20, 10))
ax1.set_tile('Source image')
ax1.imshow(img)

ax2.set_title('Warped image')
ax2.imshow(warped_im)








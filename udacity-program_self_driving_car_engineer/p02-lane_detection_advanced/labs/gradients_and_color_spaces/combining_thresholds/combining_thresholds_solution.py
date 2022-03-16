"""
# !/usr/bin/env python
# -*- coding: utf-8 -*-
@Time        : 2022/2/25 15:02
@Author      : shengdl999links@gmail.com
@ProjectName : udacity-program_self_driving_car_engineer
@File        : combining_thresholds.py
"""


import numpy as np
import cv2
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import pickle



def abs_sobel_thresh(img, orient='x', sobel_kernel=3, thresh=(0, 255)):
    # Calculate directional gradient
    # Apply threshold
    if orient == 'x':
        abs_sobel = np.absolute(cv2.Sobel(img, cv2.CV_64F, 1, 0))
    if orient == 'y':
        abs_sobel = np.absolute(cv2.Sobel(img, cv2.CV_64F, 0, 1))

    # Convert the absolute value image to 8-bit:
    scaled_sobel = np.uint8(255 * abs_sobel / np.max(abs_sobel))

    grad_binary = np.zeros_like(scaled_sobel)
    grad_binary[(scaled_sobel >= thresh[0]) & (scaled_sobel <= thresh[1])] = 1
    return grad_binary

def mag_thresh(image, sobel_kernel=3, mag_thresh=(0, 255)):
    # Calculate gradient magnitude
    # Apply threshold
    sobelx = cv2.Sobel(image, cv2.CV_64F, 1, 0, ksize=sobel_kernel)
    sobely = cv2.Sobel(image, cv2.CV_64F, 0, 1, ksize=sobel_kernel)

    # 3) Calculate the magnitude
    gradmag = np.sqrt( sobelx ** 2 + sobely ** 2 )

    # 4) Scale to 8-bit (0 - 255) and convert to type = np.uint8
    gradmag = np.uint8(255 * gradmag / np.max(gradmag))

    # 5) Create a binary mask where mag thresholds are met
    mag_binary = np.zeros_like(gradmag)
    mag_binary[(gradmag >= mag_thresh[0]) & (gradmag <= mag_thresh[1])] = 1

    return mag_binary

def dir_threshold(image, sobel_kernel=3, thresh=(0, np.pi/2)):
    # Calculate gradient direction
    # Apply threshold
    # 2) Take the gradient in x and y separately
    sobel_x = cv2.Sobel(image, cv2.CV_64F, 0, 1, ksize=sobel_kernel)
    sobel_y = cv2.Sobel(image, cv2.CV_64F, 1, 0, ksize=sobel_kernel)

    # 3) Take the absolute value of the x and y gradients
    abs_sobel_x = np.absolute(sobel_x)
    abs_sobel_y = np.absolute(sobel_y)

    # 4) Use np.arctan2(abs_sobely, abs_sobelx) to calculate the direction of the gradient
    abs_grad_dir = np.arctan2(abs_sobel_x, abs_sobel_y)

    # 5) Create a binary mask where direction thresholds are met
    dir_binary = np.zeros_like(abs_grad_dir)
    dir_binary[(abs_grad_dir >= thresh[0]) & (abs_grad_dir <= thresh[1])] = 1

    return dir_binary

# Read in an image
img = mpimg.imread('signs_vehicles_xygrad.png')
gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)

# Choose a Sobel kernel size
ksize = 10 # Choose a larger odd number to smooth gradient measurements

# Apply each of the thresholding functions
gradx = abs_sobel_thresh(gray, orient='x', sobel_kernel=ksize, thresh=(30, 100))
grady = abs_sobel_thresh(gray, orient='y', sobel_kernel=ksize, thresh=(30, 100))
mag_binary = mag_thresh(gray, sobel_kernel=9, mag_thresh=(30, 100))
dir_binary = dir_threshold(gray, sobel_kernel=15, thresh=(0.7, 1.3))

combined = np.zeros_like(dir_binary)
combined[((gradx == 1) & (grady == 1)) | ((mag_binary == 1) & (dir_binary == 1))] = 1


# Plot the result
f, (ax1, ax2) = plt.subplots(1, 2, figsize=(24, 9))
f.tight_layout()
ax1.imshow(img)
ax1.set_title('Original Image', fontsize=50)

ax2.imshow(combined, cmap='gray')
ax2.set_title('Combining Thresholds.', fontsize=50)
plt.subplots_adjust(left=0., right=1, top=0.9, bottom=0.)
plt.show()



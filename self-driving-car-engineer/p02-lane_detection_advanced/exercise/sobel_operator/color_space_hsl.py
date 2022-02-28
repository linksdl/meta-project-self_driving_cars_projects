"""
# !/usr/bin/env python
# -*- coding: utf-8 -*-
@Time        : 2022/2/25 15:20
@Author      : shengdl999links@gmail.com
@ProjectName : self-driving-car-engineer
@File        : color_space_hsl.py
"""


import numpy as np
import cv2
import matplotlib.pyplot as plt
import matplotlib.image as mpimg



image = mpimg.imread('test6.jpeg')
thresh = (180, 255)
gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
# plt.imshow(gray)
# plt.show()
binary = np.zeros_like(gray)
binary[(gray > thresh[0]) & (gray <= thresh[1])] = 1

# Plot the result
# f, (ax1, ax2) = plt.subplots(1, 2, figsize=(24, 9))
# f.tight_layout()
# ax1.imshow(gray)
# ax1.set_title('Gray', fontsize=50)
#
# ax2.imshow(binary, cmap='gray')
# ax2.set_title('Gray Binary.', fontsize=50)
# plt.subplots_adjust(left=0., right=1, top=0.9, bottom=0.)
# plt.show()

R = image[:,:,0]
G = image[:,:,1]
B = image[:,:,2]
plt.imshow(R, cmap='gray')
# plt.imshow(G, cmap='gray')
# plt.imshow(B, cmap='gray')
# plt.show()

thresh = (200, 255)
binary = np.zeros_like(R)
binary[(R > thresh[0]) & (R <= thresh[1])] = 1
plt.imshow(binary, cmap='gray')
plt.show()


hls = cv2.cvtColor(image, cv2.COLOR_RGB2HLS)
H = hls[:,:,0]
L = hls[:,:,1]
S = hls[:,:,2]


thresh = (90, 255)
binary = np.zeros_like(S)
binary[(S > thresh[0]) & (S <= thresh[1])] = 1


thresh = (15, 100)
binary = np.zeros_like(H)
binary[(H > thresh[0]) & (H <= thresh[1])] = 1

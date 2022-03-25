"""
# !/usr/bin/env python
# -*- coding: utf-8 -*-
@Time        : 2022/3/25 20:05
@File        : color_selection.py
"""


import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as np


# Read in the image and print out some stats
image = mpimg.imread('test.jpeg')
print('This image is: ', type(image), 'with dimensions:', image.shape)


# Grab the x and y size and make a copy of the image
ysize = image.shape[0]
xsize = image.shape[1]

# Note: always make a copy rather than simply using "="
color_select = np.copy(image)

# Define our color selection criteria
# Note: if you run this code, you'll find these are not sensible values!!
# But you'll get a chance to play with them soon in a quiz
# Next I define a color threshold in the variables red_threshold, green_threshold,
# and blue_threshold and populate rgb_threshold with these values.
# This vector contains the minimum values for red, green, and blue (R,G,B) that I will allow in my selection.

red_threshold = 200
green_threshold = 200
blue_threshold = 200
rgb_threshold = [red_threshold, green_threshold, blue_threshold]

# Identify pixels below the threshold
thresholds = (image[:,:,0] < rgb_threshold[0]) \
            | (image[:,:,1] < rgb_threshold[1]) \
            | (image[:,:,2] < rgb_threshold[2])
color_select[thresholds] = [0, 0, 0]

# Display the image
plt.imshow(color_select)
plt.show()

# Uncomment the following code if you are running the code locally and wish to save the image
mpimg.imsave("test-after.png", color_select)




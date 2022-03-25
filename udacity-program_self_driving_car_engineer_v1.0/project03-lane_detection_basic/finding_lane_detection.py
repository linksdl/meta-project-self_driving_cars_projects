"""
# !/usr/bin/env python
# -*- coding: utf-8 -*-
@Time        : 2022/3/25 22:42
@File        : finding_lane_detection.py
"""

import numpy as np
import cv2
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from finding_line import Line


def finding_lane_pipe(color_image, solid_lines=True):
    """
    车道检测流水线
    This function take as input a color road frame and tries to infer the lane lines in the image.
    :param color_image: input frame
    :param solid_lines: if True, only selected lane lines are returned. If False, all candidate lines are returned.
    :return: list of (candidate) lane lines.
    """

    # step1  resize to 960 x 540
    color_image = cv2.resize(color_image, (960, 540))

    # step2 convert to gray image
    gray_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)

    # step3 perform gaussian blur
    # kernel_size for gaussian blur
    kernel_size = 17
    blur_image = cv2.GaussianBlur(gray_image, (kernel_size, kernel_size), 0)

    # step4 perform edge detection
    # thresholds for canny edge
    low_threshold = 50
    high_threshold = 100
    edge_image = cv2.Canny(blur_image, threshold1=low_threshold, threshold2=high_threshold)

    # step5 perform hough transform
    # constants for Hough transformation
    rho = 2  # distance resolution in pixels of the Hough grid
    theta = np.pi / 180  # angular resolution in radians of the Hough grid
    threshold = 1  # minimum number of votes (intersections in Hough grid cell)
    min_line_len = 50  # minimum number of pixels making up a line
    max_line_gap = 20  # maximum gap in pixels between connectable line segments
    detected_lines = hough_lines_detection(img=edge_image,
                                           rho=rho,
                                           theta=theta,
                                           threshold=threshold,
                                           min_line_len=min_line_len,
                                           max_line_gap=max_line_gap)

    # line_image = np.zeros((edge_image.shape[0], edge_image.shape[1], 3), dtype=np.uint8)
    # for line in detected_lines:
    #     for x1, y1, x2, y2 in line:
    #         cv2.line(line_image, (x1, y1), (x2, y2), color=[255, 0, 0], thickness=2)

    # convert (x1, y1, x2, y2) tuples into Lines
    detected_lines = [Line(l[0][0], l[0][1], l[0][2], l[0][3]) for l in detected_lines]

    # if 'solid_lines' infer the two lane lines
    if solid_lines:
        candidate_lines = []
        for line in detected_lines:
            # consider only lines with slope between 30 and 60 degrees
            if 0.5 <= np.abs(line.slope) <= 2:
                candidate_lines.append(line)
        # interpolate lines candidates to find both lanes
        lane_lines = compute_lane_from_candidates(candidate_lines, gray_image.shape)
    else:
        # if not solid_lines, just return the hough transform output
        lane_lines = detected_lines

    return lane_lines


def region_of_interest(img, vertices):
    """
    区域选择
    Only keeps the region of the image defined by the polygon
    formed from `vertices`. The rest of the image is set to black.
    """

    # defining a blank mask to start with
    mask = np.zeros_like(img)

    # defining a 3 channel or 1 channel color to fill the mask with depending on the input image
    if len(img.shape) > 2:
        channel_count = img.shape[2]  # i.e. 3 or 4 depending on your image
        ignore_mask_color = (255,) * channel_count
    else:
        ignore_mask_color = 255

    # filling pixels inside the polygon defined by "vertices" with the fill color
    cv2.fillPoly(mask, vertices, ignore_mask_color)

    # returning the image only where mask pixels are nonzero
    masked_image = cv2.bitwise_and(img, mask)

    return masked_image, mask


def hough_lines_detection(img, rho, theta, threshold, min_line_len, max_line_gap):
    """
    `img` should be the output of a Canny transform.
    """
    lines = cv2.HoughLinesP(img, rho, theta, threshold, np.array([]), minLineLength=min_line_len,
                            maxLineGap=max_line_gap)
    return lines


def weighted_img(img, initial_img, α=0.8, β=1., λ=0.):
    """
    Returns resulting blend image computed as follows:

    initial_img * α + img * β + λ
    """
    img = np.uint8(img)
    if len(img.shape) == 2:
        img = np.dstack((img, np.zeros_like(img), np.zeros_like(img)))

    return cv2.addWeighted(initial_img, α, img, β, λ)


def compute_lane_from_candidates(line_candidates, img_shape):
    """
    Compute lines that approximate the position of both road lanes.

    :param line_candidates: lines from hough transform
    :param img_shape: shape of image to which hough transform was applied
    :return: lines that approximate left and right lane position
    """

    # separate candidate lines according to their slope
    pos_lines = [l for l in line_candidates if l.slope > 0]
    neg_lines = [l for l in line_candidates if l.slope < 0]

    # interpolate biases and slopes to compute equation of line that approximates left lane
    # median is employed to filter outliers
    neg_bias = np.median([l.bias for l in neg_lines])
    neg_slope = np.median([l.slope for l in neg_lines])
    x1, y1 = 0, neg_bias
    x2, y2 = -np.int32(np.round(neg_bias / neg_slope)), 0
    left_lane = Line(x1, y1, x2, y2)

    # interpolate biases and slopes to compute equation of line that approximates right lane
    # median is employed to filter outliers
    lane_right_bias = np.median([l.bias for l in pos_lines])
    lane_right_slope = np.median([l.slope for l in pos_lines])
    x1, y1 = 0, lane_right_bias
    print(lane_right_bias, lane_right_slope)
    # x2, y2 = np.int32(np.round((img_shape[0] - lane_right_bias) / lane_right_slope)), img_shape[0]
    x2, y2 = np.int32((img_shape[0] - lane_right_bias) / lane_right_slope), img_shape[0]
    right_lane = Line(x1, y1, x2, y2)

    return left_lane, right_lane


def smoothen_over_time(lane_lines):
    """
    Smooth the lane line inference over a window of frames and returns the average lines.
    """

    avg_line_lt = np.zeros((len(lane_lines), 4))
    avg_line_rt = np.zeros((len(lane_lines), 4))

    for t in range(0, len(lane_lines)):
        avg_line_lt[t] += lane_lines[t][0].get_coords()
        avg_line_rt[t] += lane_lines[t][1].get_coords()

    return Line(*np.mean(avg_line_lt, axis=0)), Line(*np.mean(avg_line_rt, axis=0))


def color_frame_pipeline(frames, solid_lines=False, temporal_smoothing=True):
    """
    Entry point for lane detection pipeline. Takes as input a list of frames (RGB) and returns an image (RGB)
    with overlaid the inferred road lanes. Eventually, len(frames)==1 in the case of a single image.
    """
    is_videoclip = len(frames) > 0

    img_h, img_w = frames[0].shape[0], frames[0].shape[1]

    lane_lines = []
    for t in range(0, len(frames)):
        inferred_lanes = finding_lane_pipe(color_image=frames[t], solid_lines=solid_lines)
        lane_lines.append(inferred_lanes)

    if temporal_smoothing and solid_lines:
        lane_lines = smoothen_over_time(lane_lines)
    else:
        lane_lines = lane_lines[0]

    # prepare empty mask on which lines are drawn
    line_img = np.zeros(shape=(img_h, img_w))

    # draw lanes found
    for lane in lane_lines:
        lane.draw(line_img)

    # keep only region of interest in masking
    imshape = (img_h, img_w)
    # vertices = np.array([[(50, img_h),
    #                       (450, 320),
    #                       (490, 320),
    #                       (img_w - 50, img_h)]],
    #                     dtype=np.int32)
    vertices = np.array([[(0, imshape[0]), (450, 320), (490, 320), (imshape[1], imshape[0])]], dtype=np.int32)
    img_masked, _ = region_of_interest(line_img, vertices)

    # make blend on color image
    img_color = frames[-1] if is_videoclip else frames[0]
    img_blend = weighted_img(img_masked, img_color, α=0.8, β=1., λ=0.)

    return img_blend

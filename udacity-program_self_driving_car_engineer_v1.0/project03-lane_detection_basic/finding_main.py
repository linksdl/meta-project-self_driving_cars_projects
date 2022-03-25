"""
# !/usr/bin/env python
# -*- coding: utf-8 -*-
@Time        : 2022/3/25 23:21
@File        : finding_main.py
"""

import matplotlib.pyplot as plt
import cv2
import os
from os.path import join, basename
from collections import deque
from finding_lane_detection import color_frame_pipeline

resize_h, resize_w = 540, 960


def test_images(in_image_dir, out_image_dir):
    """
    测试在图片上
    """
    test_images = [join(in_image_dir, name) for name in os.listdir(in_image_dir)]

    for image in test_images:
        # print('Processing image: {}'.format(image))

        in_image = cv2.cvtColor(cv2.imread(image, cv2.IMREAD_COLOR), cv2.COLOR_BGR2RGB)
        out_image = color_frame_pipeline([in_image], solid_lines=True)

        out_image_path = join(out_image_dir, basename(image))
        print(out_image_path)
        cv2.imwrite(out_image_path, cv2.cvtColor(out_image, cv2.COLOR_RGB2BGR))

    plt.close('all')


# testing on images
# test_images('test_images', 'test_images_out')


def test_videos(in_videos_dir, out_videos_dir):
    test_videos = [join(in_videos_dir, name) for name in os.listdir(in_videos_dir)]

    for video in test_videos:
        print('Processing video: {}'.format(video))

        cap = cv2.VideoCapture(video)
        out_videos_path = join(out_videos_dir, basename(video))
        print(out_videos_path)
        out = cv2.VideoWriter(out_videos_path,
                              fourcc=cv2.VideoWriter_fourcc(*'DIVX'),
                              fps=20.0, frameSize=(resize_w, resize_h))

        frame_buffer = deque(maxlen=10)
        while cap.isOpened():
            ret, color_frame = cap.read()
            if ret:
                color_frame = cv2.cvtColor(color_frame, cv2.COLOR_BGR2RGB)
                color_frame = cv2.resize(color_frame, (resize_w, resize_h))
                frame_buffer.append(color_frame)
                blend_frame = color_frame_pipeline(frames=frame_buffer, solid_lines=False)
                out.write(cv2.cvtColor(blend_frame, cv2.COLOR_RGB2BGR))
                cv2.imshow('blend', cv2.cvtColor(blend_frame, cv2.COLOR_RGB2BGR)), cv2.waitKey(1)
            else:
                break
        cap.release()
        out.release()
        cv2.destroyAllWindows()


# testing on videos
test_videos('test_videos', 'test_videos_out')

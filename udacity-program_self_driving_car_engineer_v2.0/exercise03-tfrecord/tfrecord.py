"""
# !/usr/bin/env python
# -*- coding: utf-8 -*-
@Time        : 2022/2/24 12:35
@Author      : shengdl999links@gmail.com
@ProjectName : udacity-program_self_driving_car_engineer_v2.0
@File        : tfrecord.py
"""
import os
import numpy as np
import tensorflow as tf

import tempfile
example_path = os.path.join(tempfile.gettempdir(), "example.tfrecords")
np.random.seed(0)

# write the records to a file

with tf.io.TFRecordWriter(example_path) as file_writer:
    for _ in range(10):
        x, y = np.random.random(), np.random.random()

        record_bytes = tf.train.Example(features=tf.train.Features(
            feature={ "x": tf.train.Feature(float_list = tf.train.FloatList(value=[x])),
                      "y": tf.train.Feature(float_list = tf.train.FloatList(value=[y]))}
        )).SerializeToString()
        file_writer.write(record_bytes)


# Read the data back out.
def decode_fn(record_bytes):
  return tf.io.parse_single_example(
      # Data
      record_bytes,
      # Schema
      {"x": tf.io.FixedLenFeature([], dtype=tf.float32),
       "y": tf.io.FixedLenFeature([], dtype=tf.float32)}
  )


for batch in tf.data.TFRecordDataset([example_path]).map(decode_fn):
  print("x = {x:.4f},  y = {y:.4f}".format(**batch))




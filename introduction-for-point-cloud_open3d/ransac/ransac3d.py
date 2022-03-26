"""
# !/usr/bin/env python
# -*- coding: utf-8 -*-
@Time        : 2022/3/15 09:39
@Author      : shengdl999links@gmail.com
@ProjectName : introduction-for-point-cloud_open3d_v1
@File        : ransac3d.py
"""

import open3d as o3d



pcd = o3d.io.read_point_cloud("data/simpleHighway.pcd")
plane_model, inliers = pcd.segment_plane(distance_threshold=0.1,
                                         ransac_n=10,
                                         num_iterations=1000)
[a, b, c, d] = plane_model
print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")

inlier_cloud = pcd.select_by_index(inliers)
inlier_cloud.paint_uniform_color([1.0, 0, 0])
outlier_cloud = pcd.select_by_index(inliers, invert=True)
outlier_cloud.paint_uniform_color([0, 1, 0])
o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud])

"""
# !/usr/bin/env python
# -*- coding: utf-8 -*-
@Time        : 2022/3/6 21:53
@Author      : shengdl999links@gmail.com
@ProjectName : introduction-for-point-cloud-open3d_v1
@File        : deer_demo.py
"""


import open3d as o3d

house = o3d.io.read_point_cloud("deer.ply")
print(house)

o3d.visualization.draw_geometries([house])

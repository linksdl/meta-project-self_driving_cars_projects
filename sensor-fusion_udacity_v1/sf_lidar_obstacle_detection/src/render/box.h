#ifndef BOX_H
#define BOX_H

#include <Eigen/Geometry>

// https://codextechnicanum.blogspot.com/2015/04/find-minimum-oriented-bounding-box-of.html
struct  BoxQ
{
    /* data */
    Eigen::Vector3f  bboxTransform;
    Eigen::Quaternionf bboxQuternion;

    float cube_length;
    float cube_width;
    float cube_height;
};

struct Box
{
    /* data */

    float x_min;
    float y_min;
    float  z_min;
    float x_max;
    float y_max;
    float z_max;
};

#endif
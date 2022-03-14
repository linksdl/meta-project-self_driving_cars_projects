// Functions and structs used to render the enviroment
// such as cars and the highway

#ifndef RENDER_H
#define RENDER_H

#include <pcl/visualization/pcl_visualizer.h>
#include "box.h"
#include "iostream"
#include <vector>
#include <string>

struct Color
{
    float r, g, b;

    Color(float setR, float setG, float setB) : r(setR), g(setG), b(setB) {}
};

struct Vect3
{
    double x, y, z;

    Vect3(double setX, double setY, double setZ) : x(setX), y(setY), z(setZ) {}

    Vect3 operator+(const Vect3 &vec)
    {
        Vect3 result(x + vec.x, y + vec.y, z + vec.z);
        return result;
    }

};

enum CameraAngle
{
    XY, TopDown, Side, FPS

};

struct Car
{
    // units in meters;
    Vect3 position, dimensions;

    std::string name;
        Color color;

  	Car(Vect3 setPosition, Vect3 setDimensions, Color setColor, std::string setName)
    	: position(setPosition), dimensions(setDimensions), color(setColor), name(setName){}

        void render(pcl::visualization::PCLVisualizer::Ptr& viewer)
        {
		// render bottom of car
		viewer->addCube(position.x-dimensions.x/2, position.x+dimensions.x/2, position.y-dimensions.y/2, position.y+dimensions.y/2, position.z, position.z+dimensions.z*2/3, color.r, color.g, color.b, name); 
      	viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, name); 
        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b, name);
        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 1.0, name);
		// render top of car
		viewer->addCube(position.x-dimensions.x/4, position.x+dimensions.x/4, position.y-dimensions.y/2, position.y+dimensions.y/2, position.z+dimensions.z*2/3, position.z+dimensions.z, color.r, color.g, color.b, name+"Top"); 
      	viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, name+"Top"); 
        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b, name+"Top");
        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 1.0, name+"Top");

        }

};

void renderHighway(pcl::visualization::PCLVisualizer::Ptr& viewer);

#endif
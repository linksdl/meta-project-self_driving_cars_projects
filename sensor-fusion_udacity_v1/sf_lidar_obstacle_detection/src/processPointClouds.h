// PCL lib Functions for processing point clouds 

#ifndef PROCESSPOINTCLOUDS_H_
#define PROCESSPOINTCLOUDS_H_

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>

// pcl for filiter
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>

// kd-tree
#include <pcl/kdtree/kdtree.h>

// segmentation
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>

// 
#include <iostream>
#include <string>
#include <vector>
#include <ctime>
#include <chrono>
#include "render/box.h"

template <typename PointT>
class ProcessPointClouds 
{
    public:
            // constructor
            ProcessPointClouds();

            // deconstructor
            ~ProcessPointClouds();


            // get the number of the points clouds
            void  numPoints(typename pcl::PointCloud<PointT>::Ptr cloud);

            // filter
            typename pcl::PointCloud<PointT>::Ptr FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint);

            // separate 
            std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud);

            // segment the road plane
            std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold);

            // cludstering
             std::vector<typename pcl::PointCloud<PointT>::Ptr> Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize);

            // load, save PCD files
            void savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file);
            typename pcl::PointCloud<PointT>::Ptr loadPcd(std::string, file);
            std::vector<boost::filesystem::path> streamPcd(std::string dataPath);

            // Bounding box
            Box BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster);

}


#endif
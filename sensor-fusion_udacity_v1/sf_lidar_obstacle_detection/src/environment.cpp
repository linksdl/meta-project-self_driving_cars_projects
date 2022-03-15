/*
  * Create simple 3d highway enviroment using PCL  
  * for exploring self-driving car sensors
  * */

 #include "sensors/lidar.h"
 #include "render/render.h"


#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"


std::vector<Car> initHighway(bool  renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
 {
    Car egoCar(Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");

  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
      renderHighway(viewer);
      egoCar.render(viewer);
      car1.render(viewer);
      car2.render(viewer);
      car3.render(viewer);
    }

    return cars;
}


// init the camera
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    viewer->setBackgroundColor(0,0,0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;

    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle != FPS)
    {
      viewer-> addCoordinateSystem(1.0);
    }

}


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer, bool renderScene, bool render_obst, bool  render_plane, bool render_box)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    // bool renderScene = true;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // TODO:: Create lidar sensor 
    Lidar* lidar = new Lidar(cars, 0);
    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud = lidar->scan();
    // renderRays(viewer, lidar->position, inputCloud);
    renderPointCloud(viewer, inputCloud, "inputCloud");

    // TODO:: Create point processor
    ProcessPointClouds<pcl::PointXYZ> pointProcessor; 
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointProcessor.SegmentPlane(inputCloud, 100, 0.2);
    if(render_obst) renderPointCloud(viewer, segmentCloud.first, "obstCloud", Color(1,0,0));
    if(render_plane) renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0,1,0));

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessor.Clustering(segmentCloud.first,  1.0,  3, 30);
    std::cout<<"num of cluster: "<<cloudClusters.size()<<endl;
    int clusterId = 0;
    std::vector<Color> colors = {Color(1, 0,0), Color(0,1,0), Color(0,0,1)};

    for (pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
    {
      //std::cout << "cluster size ";
      std::cout<<"size of cluster"+std::to_string(clusterId)<<":";
      pointProcessor.numPoints(cluster);
      renderPointCloud(viewer, cluster, "obcluster"+std::to_string(clusterId), colors[clusterId % colors.size()]);
      //renderPointCloud(viewer, cluster, "obstcloud"+std::to_string(clusterId), colors[clusterId]);

    if(render_box)
    {
       Box box = pointProcessor.BoundingBox(cluster);
       renderBox(viewer, box, clusterId);
    }
      ++ clusterId;
    }

  renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0,1,0));
}

int main(int agrc, char** argv)
{
  std::cout << "starting the environment " <<  std::endl;

  pcl::visualization::PCLVisualizer::Ptr viewer( new pcl::visualization::PCLVisualizer ("3D Highway"));
  CameraAngle setAngle = XY;
  initCamera (setAngle, viewer);
  bool renderScene = false;
  bool render_obst = true;
  bool render_plane = true;
  bool render_box = true;
  simpleHighway(viewer, renderScene, render_obst, render_plane, render_box);

  while (!viewer->wasStopped() )
  {
    viewer->spinOnce();
  }
}



/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
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

void cityBlockDisplay(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
  bool renderScene = false;
  bool render_obst = true;
  bool render_plane = false;
  bool render_clusters = true;
  bool render_box = true;
  ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
  pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessorI->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
  pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud = pointProcessorI->FilterCloud(inputCloud, 0.3, Eigen::Vector4f (-10, -5, -2, 1), Eigen::Vector4f ( 30, 8, 1, 1));
  //renderPointCloud(viewer, inputCloud, "inputCloud");
  if(renderScene)
    renderPointCloud(viewer, filterCloud, "inputCloud");

  //std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessorI->SegmentPlane(inputCloud, 100, 0.3);
  std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessorI->RansacPlane(filterCloud, 100, 0.3);

  pcl::PointCloud<pcl::PointXYZI>::Ptr obst = segmentCloud.first;
  float min_z = 100, max_z=0;

  for(size_t i = 0;i<obst->points.size();i++)
  {
    if(min_z>obst->points[i].z) min_z = obst->points[i].z;
    if(max_z<obst->points[i].z) max_z = obst->points[i].z;
  }
  for(size_t i = 0;i<obst->points.size();i++)
  {
    obst->points[i].z = min_z;
  }
  int old_size = obst->points.size();
  obst->points.resize(2*old_size);
  for(size_t i = old_size;i<obst->points.size();i++)
  {
    obst->points[i].x = obst->points[i-old_size].x;
    obst->points[i].y = obst->points[i-old_size].y;
    obst->points[i].z = max_z;
  }

  if(render_obst)
    renderPointCloud(viewer, segmentCloud.first, "obstcloud", Color(1,0,0));
  if(render_plane)
    renderPointCloud(viewer, segmentCloud.second, "planecloud", Color(0,1,0));

}


//void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer)
void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointProcessorI, pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
{
  // RENDER OPTIONS
  bool renderScene = false;
  bool render_obst = true;
  bool render_plane = true;
  bool render_clusters = true;
  bool render_box = true;
  //ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
  //pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessorI->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
  inputCloud = pointProcessorI->FilterCloud(inputCloud, 0.3, Eigen::Vector4f (-10, -5, -2, 1), Eigen::Vector4f ( 40, 7, 1, 1));
  if(renderScene)
    renderPointCloud(viewer, inputCloud, "inputCloud");

  //std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessorI->SegmentPlane(inputCloud, 100, 0.3);
  std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessorI->RansacPlane(inputCloud, 100, 0.3);
  if(render_obst)
    renderPointCloud(viewer, segmentCloud.first, "obstcloud", Color(1,0,0));
  if(render_plane)
    renderPointCloud(viewer, segmentCloud.second, "planecloud", Color(0,1,0));

  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->Clustering(segmentCloud.first, 0.5, 5, 400);
  int clusterId = 0;
  std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1)};

  for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
  {
    if(render_clusters){
      std::cout << "cluster size ";
      pointProcessorI->numPoints(cluster);
      renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId%colors.size()]);
    }
    if(render_box)
    {
      Box box = pointProcessorI->BoundingBox(cluster);
      renderBox(viewer, box, clusterId);
    }
    ++clusterId;
  }
}
void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------

    // RENDER OPTIONS
    bool renderScene = false;
    bool render_obst = false;
    bool render_plane = false;
    bool render_clusters = true;
    bool render_box = true;
    std::vector<Car> cars = initHighway(renderScene, viewer);

    // TODO:: Create lidar sensor
    Lidar* lidar = new Lidar(cars, 0);
    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud = lidar->scan();
    //renderRays(viewer, lidar->position, inputCloud);
    //renderPointCloud(viewer,inputCloud,"inputCloud");
    // TODO:: Create point processor
    ProcessPointClouds<pcl::PointXYZ> pointProcessor;
    //ProcessPointClouds<pcl::PointXYZ> pointProcessor = new ProcessPointClouds<pcl::PointXYZ>();

    //std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointProcessor.SegmentPlane(inputCloud, 100, 0.3);
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointProcessor.RansacPlane(inputCloud, 100, 0.3);

    if(render_obst)
      renderPointCloud(viewer, segmentCloud.first, "obstcloud", Color(1,0,0));
    if(render_plane)
      renderPointCloud(viewer, segmentCloud.second, "planecloud", Color(0,1,0));
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessor.Clustering(segmentCloud.first, 1.0, 3, 30);
    //{
    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1)};

    for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
     {
       if(render_clusters){
       std::cout << "cluster size ";
       pointProcessor.numPoints(cluster);
       renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId%colors.size()]);
     }
     if(render_box)
     {
      Box box = pointProcessor.BoundingBox(cluster);
      renderBox(viewer, box, clusterId);
     }
      ++clusterId;
    }
     //}
}


//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);

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

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    //simpleHighway(viewer);
    //cityBlock(viewer);
    cityBlockDisplay(viewer);
    while(!viewer->wasStopped ())
    {
      viewer->spinOnce ();
    }
/*
    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI;
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

    while (!viewer->wasStopped ())
    {
      viewer->removeAllPointClouds();
      viewer->removeAllShapes();

      inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
      cityBlock(viewer, pointProcessorI, inputCloudI);

      streamIterator++;
      if(streamIterator==stream.end())
        streamIterator=stream.begin();

      viewer->spinOnce ();
    }*/
}

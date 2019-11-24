/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors


#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"
#include "ransac_3d.h"
#include "cluster_3d.cpp"

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer,
               ProcessPointClouds<pcl::PointXYZI>* pointProcessorI,
               pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloudI)
{
  // RENDER OPTIONS
  bool renderScene = true;
  bool render_obst = true;
  bool render_plane = true;
  bool render_clusters = true;
  bool render_box = true;
  /* Down sample point cloud to speed up the processing speed*/
  pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud;
  inputCloud = pointProcessorI->FilterCloud(inputCloudI, 0.3,
                                            Eigen::Vector4f (-10, -6, -2, 1),
                                            Eigen::Vector4f ( 30, 8, 1, 1));
  inputCloudI = pointProcessorI->FilterCloud(inputCloudI, 0.001,
                                            Eigen::Vector4f (-10, -6, -2, 1),
                                            Eigen::Vector4f ( 30, 8, 1, 1));
  if(renderScene)
    renderPointCloud(viewer, inputCloudI, "inputCloud");
  /* Remove the points from ground plane, the remaining points are taken as obstacle,
     later processing are down on those obstale points to reduce computation time*/
  Ransac_3D<pcl::PointXYZI> RansacP;
  std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr,
            pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = RansacP.RansacPlane(inputCloud, 100, 0.3);

  if(render_obst)
    renderPointCloud(viewer, segmentCloud.first, "obstcloud", Color(1,0,0));
  if(render_plane)
    renderPointCloud(viewer, segmentCloud.second, "planecloud", Color(0,1,0));
  /*Clustering are implemented from scratch*/
  Cluster_3D<pcl::PointXYZI> ClusterP;
  //std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters
  //                                                 = pointProcessorI->Clustering(segmentCloud.first, 0.5, 5, 400);
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters
                                                  = ClusterP.euclideanCluster(segmentCloud.first, 0.5,5,400);
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
      BoxQ box = pointProcessorI->BoundingBoxQ(cluster);
      //Box box = pointProcessorI->BoundingBox(cluster);
      renderBox(viewer, box, clusterId);
    }
    ++clusterId;
  }
}

//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle,
                pcl::visualization::PCLVisualizer::Ptr& viewer)
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

    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI;
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_2");
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
    }
}

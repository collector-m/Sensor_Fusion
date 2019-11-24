#include <pcl/common/common.h>
#include <unordered_set>
#include <chrono>

template<typename PointT>
struct Ransac_3D{
  Ransac_3D(){}
  //template<typename PointT>
  std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr>
                         RansacPlane(typename pcl::PointCloud<PointT>::Ptr& cloud,
                                    int maxIterations,
                                    float distanceTol)
  {
  	std::unordered_set<int> inliersResult;
  	srand(time(NULL));

  	auto startTime = std::chrono::steady_clock::now();
  	while(maxIterations--)
  	{
  		std::unordered_set<int> inliers;
  		while(inliers.size()<3)
  			inliers.insert(rand()%(cloud->points.size()));
  		float x1,y1,z1, x2,y2,z2,x3,y3,z3;
  		auto itr = inliers.begin();
      x1 = cloud->points[*itr].x;
  		y1 = cloud->points[*itr].y;
  		z1 = cloud->points[*itr].z;
  		itr++;
  		x2 = cloud->points[*itr].x;
  		y2 = cloud->points[*itr].y;
  		z2 = cloud->points[*itr].z;
  		itr++;
  		x3 = cloud->points[*itr].x;
  		y3 = cloud->points[*itr].y;
  		z3 = cloud->points[*itr].z;

  	  float a = (y2 - y1) * (z3 - z1) - (z2 - z1) * (y3 - y1);
      float b = (z2 - z1) * (x3 - x1) - (x2 - x1) * (z3 - z1);
  		float c = (x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1);
      float d = -(a * x1 + b * y1 + c * z1);

  		for(int index = 0; index<cloud->points.size(); index++)
  		{
  			if(inliers.count(index)>0)
  			  continue;
  			PointT point = cloud->points[index];
        float x_ = point.x;
  			float y_ = point.y;
  			float z_ = point.z;
  			float dis = fabs(a * x_ + b * y_ + c * z_ + d)/sqrt(a*a + b*b + c*c);
  			if(dis<=distanceTol)
  			  inliers.insert(index);
  		}
  		if(inliers.size()>inliersResult.size())
  		{
  				inliersResult = inliers;
  		}
  	}

    typename pcl::PointCloud<PointT>::Ptr  cloudInliers(new pcl::PointCloud<PointT>());
  	typename pcl::PointCloud<PointT>::Ptr cloudOutliers(new pcl::PointCloud<PointT>());

  	for(int index = 0; index < cloud->points.size(); index++)
  	{
  		PointT point = cloud->points[index];
  		if(inliersResult.count(index))
  			cloudInliers->points.push_back(point);
  		else
  			cloudOutliers->points.push_back(point);
  	}
    std::pair<typename pcl::PointCloud<PointT>::Ptr,
              typename pcl::PointCloud<PointT>::Ptr> segResult(cloudOutliers,cloudInliers);

  	auto endTime = std::chrono::steady_clock::now();
  	auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
  	std::cout<<"RANSAC_PLANE took "<<elapsedTime.count()<<" milliseconds"<<std::endl;
    return segResult;
  }
};

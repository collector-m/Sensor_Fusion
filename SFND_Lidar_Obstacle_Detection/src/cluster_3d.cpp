/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include <chrono>
#include <string>
#include "kdtree_3d.h"

template<typename PointT>
struct Cluster_3D{

	void clusterHelper(int indice,
		                 typename pcl::PointCloud<PointT>::Ptr& points,
										 typename pcl::PointCloud<PointT>::Ptr& cluster,
										 std::vector<bool>& processed,
										 KdTree<PointT>* tree,
										 float distanceTol)
	{
		processed[indice] = true;
		cluster->points.emplace_back(points->points[indice]);
		std::vector<int> nearest = tree->search(points->points[indice],distanceTol);
		for(int id:nearest)
		{
			if(!processed[id])
	      clusterHelper(id, points, cluster, processed, tree, distanceTol);
		}
	}


std::vector<typename pcl::PointCloud<PointT>::Ptr>
                 euclideanCluster(typename pcl::PointCloud<PointT>::Ptr& points,
																	float distanceTol,int minSize,int maxSize)
	{
		// TODO: Fill out this function to return list of indices for each cluster

		std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
		std::vector<bool> processed(points->points.size(),false);
		KdTree<PointT>* tree = new KdTree<PointT>();
		for(int i=0;i<points->points.size();i++)
		{
			tree->insert(points->points[i],i);
		}

		int i=0;
		while(i<points->points.size())
		{
			if(processed[i])
			{
				i++;
				continue;
			}
			//std::vector<int> cluster;
			typename pcl::PointCloud<PointT>::Ptr cluster(new pcl::PointCloud<PointT>);
			clusterHelper(i, points, cluster, processed, tree, distanceTol);
			cluster->width = cluster->points.size();
			cluster->height = 1;
			if(cluster->points.size()>=minSize && cluster->points.size()<=maxSize)
				clusters.emplace_back(cluster);
			i++;
		}

		return clusters;
	}
};

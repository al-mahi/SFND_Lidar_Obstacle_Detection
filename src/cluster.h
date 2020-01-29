/* \author Aaron Brown */
/* modified for by S M Al Mahi for Lidar Obstacle detection project */
// Quiz on implementing kd tree
#ifndef CLUSTER_H_
#define CLUSTER_H_

#include <pcl/common/common.h>
#include <chrono>
#include <string>
#include <vector>
#include "kdtree.h"

template <typename PointT>
void clusterHelper(int indice, typename pcl::PointCloud<PointT>::Ptr cloud, std::vector<int>& cluster, std::vector<bool>& processed, KdTree<PointT>* tree, float distanceTol, int minsize, int maxsize)
{
    processed[indice] = true;
    cluster.push_back(indice);

    std::vector<int> nearest = tree->search(cloud->points[indice], distanceTol);

    for(int id: nearest)
    {
        if(!processed[id] && cluster.size() < maxsize)
            clusterHelper(id, cloud, cluster, processed, tree, distanceTol, minsize, maxsize);
    }
}

template <typename PointT>
std::vector<std::vector<int>> euclideanCluster(typename pcl::PointCloud<PointT>::Ptr cloud, KdTree<PointT>* tree, float distanceTol, int minsize, int maxsize)
{

	// TODO: Fill out this function to return list of indices for each cluster

	std::vector<std::vector<int>> clusters;

	std::vector<bool> processed(cloud->points.size(), false);

	int i = 0;
	while (i < cloud->points.size())
    {
	    if(processed[i])
        {
	        i++;
            continue;
        }
	    std::vector<int> cluster;

	    clusterHelper(i, cloud, cluster, processed, tree, distanceTol, minsize, maxsize);
	    if(cluster.size() > minsize)
	        clusters.push_back(cluster);
	    i++;
    }
	return clusters;

}


#endif /* CLUSTER_H_*/
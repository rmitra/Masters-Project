#ifndef UTIL_HPP
#define UTIL_HPP

#include <vector>
#include <pcl/kdtree/impl/kdtree_flann.hpp>

using namespace std;

bool is_occluded(pcl::KdTreeFLANN<pcl::PointXYZ> &kdtree_grid, int i, int j, int k)
{
	pcl::PointXYZ q_point;

	q_point.x = (double)i;
	q_point.y = (double)j;
	q_point.z = (double)k;

	vector<int> pointIdxRadiusSearch;
	vector<float> pointRadiusSquaredDistance;

	int found_instances = kdtree_grid.radiusSearch(q_point, 1.8, pointIdxRadiusSearch, pointRadiusSquaredDistance);

    return (found_instances < 10);
	// return (found_instances < 5);
}

#endif
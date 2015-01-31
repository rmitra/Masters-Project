#ifndef UTIL_HPP
#define UTIL_HPP

#include <vector>
#include <pcl/kdtree/impl/kdtree_flann.hpp>

using namespace std;

int num_neighbours(pcl::KdTreeFLANN<pcl::PointXYZ> &kdtree_grid, int i, int j, int k, int radius=1.8)
{
	pcl::PointXYZ q_point;

	q_point.x = (double)i;
	q_point.y = (double)j;
	q_point.z = (double)k;

	vector<int> pointIdxRadiusSearch;
	vector<float> pointRadiusSquaredDistance;

	int found = kdtree_grid.radiusSearch(q_point, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);
    return found;
}

bool is_spurious(pcl::KdTreeFLANN<pcl::PointXYZ> &kdtree_grid, int i, int j, int k)
{
    int nearer_neighbours = num_neighbours(kdtree_grid, i, j, k, 1.8);
    int farther_neighbours = num_neighbours(kdtree_grid, i, j, k, 3.6);
    float ratio = (float)farther_neighbours / (float)nearer_neighbours;

    return (ratio < 7.5);
}

#endif
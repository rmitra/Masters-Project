#ifndef HEURISTICS_HPP
#define HEURISTICS_HPP

#include <vector>
#include <pcl/kdtree/impl/kdtree_flann.hpp>

#include "block.hpp"

using namespace std;

// find the number of neighbours a point has within a distance of radius
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

// heuristic for determining if a voxel is noisy: the ratio of the number of
// neighbours it has in a distance of 2R is much less than the number of neighbours
// it has in a distance of R (here by much less we mean threshold or lower)
// default value of R is 1.8
bool is_spurious(pcl::KdTreeFLANN<pcl::PointXYZ> &kdtree_grid, int i, int j, int k, int radius=1.8, int threshold=7.5)
{
    int nearer_neighbours = num_neighbours(kdtree_grid, i, j, k, radius);
    int farther_neighbours = num_neighbours(kdtree_grid, i, j, k, 2*radius);
    float ratio = (float)farther_neighbours / (float)nearer_neighbours;

    return (ratio < threshold);
}

// for each box, only draw those v

// post processing sanity check step: shrink any boxes that do not tightly bound their contained points
// in other words, if a box has some slack space, remove it
void trim_boxes(vector<Block> &block_list)
{
    // cout << "topkek" << endl;
}

#endif
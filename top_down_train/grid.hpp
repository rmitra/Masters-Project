#ifndef GRID_HPP
#define GRID_HPP

#include <iostream>
#include <cstdio>
#include <cmath>
#include <vector>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>

#include "block.hpp"
#include "heuristics.hpp"

/* end of header files */

#define EPS 0.001

using namespace std;

// element for every grid node
class grid_element
{
public:
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_list; // list of those points
	pcl::PointCloud<pcl::Normal>::Ptr n_list; // list of normals
	double r,g,b;

	bool used;

	grid_element()
	{
		p_list = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
		n_list = pcl::PointCloud<pcl::Normal>::Ptr(new pcl::PointCloud<pcl::Normal>);
		used = false;
	}
};

class grid 
{
public:
	//	pcl::PointXYZ resolution; // dimension of a cubical voxel
	double resolution;
	grid_element *** data;
	int length, width, height; // dimensions of the grid
	pcl::PointXYZ ref_point; // here the min point is taken as the reference point
		
	grid(pcl::PointXYZ res, pcl::PointXYZ minPoint, pcl::PointXYZ maxPoint);
	grid(double res, pcl::PointXYZ minPoint, pcl::PointXYZ maxPoint);
	grid(int _length, int _width, int _height, double res, pcl::PointXYZ minPoint, pcl::PointXYZ maxPoint);

	void display_dimensions();

	void allocate_points_to_grid (pcl::PointCloud<pcl::PointXYZRGB> cloud, pcl::PointCloud<pcl::Normal> cloud_normals);
	void allocate_points_to_grid_display( pcl::PointCloud<pcl::PointXYZRGB> cloud, pcl::PointCloud<pcl::Normal> cloud_normals, double orig_resolution);

	void remove_spurious_voxels(pcl::KdTreeFLANN<pcl::PointXYZ> &kdtree_grid);
	void remove_voxels(int threshold, bool disp);
};

#endif

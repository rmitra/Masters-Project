#include <iostream>
#include <cstdio>
#include <cmath>
#include <vector>
#include <algorithm>

#include <pcl/kdtree/impl/kdtree_flann.hpp>

/* header files for PCL and grid structure */

#include "grid_datastructure.cpp"

/* start of program */

#ifndef VOXELIZATION_GRID

#define VOXELIZATION_GRID

#define DIM_FRAC 0.1f

/* function to find min max of a cloud 
 * Input is a point cloud
 * Output are two pcl::PointXYZ containg the min and the max
 */

void find_min_max(pcl::PointCloud<pcl::PointXYZRGB> cloud, pcl::PointXYZ &min, pcl::PointXYZ &max){

	double xmin = cloud.points[0].x, xmax = cloud.points[0].x;
	double ymin = cloud.points[0].y, ymax = cloud.points[0].y;
	double zmin = cloud.points[0].z, zmax = cloud.points[0].z;
	
	for(int i = 1; i< cloud.points.size(); i++){
		
		if( cloud.points[i].x < xmin )
			xmin = cloud.points[i].x;
		else if( cloud.points[i].x > xmax ) 	
			xmax = cloud.points[i].x;
	
		if( cloud.points[i].y < ymin )
			ymin = cloud.points[i].y;
		else if( cloud.points[i].y > ymax ) 	
			ymax = cloud.points[i].y;
			
		if( cloud.points[i].z < zmin )
			zmin = cloud.points[i].z;
		else if( cloud.points[i].z > zmax ) 	
			zmax = cloud.points[i].z;	
	
	}		
	
	min.x = xmin ;	min.y = ymin ;	min.z = zmin ;
	max.x = xmax ;	max.y = ymax ;	max.z = zmax ;

		
	return;
}	

/* Set up the grid
 * returns a grid*
 * Input args are resolution, minpoint and maxpoint of the cloud
 */

grid* setup_grid(pcl::PointXYZ & resolution, pcl::PointXYZ minPoint, pcl::PointXYZ maxPoint){

	grid *g = new grid(resolution, minPoint, maxPoint);
	// g->display_dimensions();
		
	return g;
}	

/* Allocates points of the cloud to particular voxels
 * remove voxels having points below a threshold
 */


void set_kdtree_grid(grid *g, pcl::KdTreeFLANN<pcl::PointXYZ> &kdtree_grid){

	pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointXYZ t_data;
	
	for(int i = 0; i < g->length; i++){
		for(int j = 0; j < g->width; j++){
			for(int k = 0; k < g->height; k++){
				if(g->data[i][j][k].used){
				
					t_data.x = i;
					t_data.y = j;
					t_data.z = k;
					
					(voxel_cloud->points).push_back(t_data);
				}
			}
		}
	}
	
	kdtree_grid.setInputCloud(voxel_cloud);
}


void voxelize(grid *g, pcl::PointCloud<pcl::PointXYZRGB> cloud, pcl::PointCloud<pcl::Normal> cloud_normals, pcl::KdTreeFLANN<pcl::PointXYZ> &kdtree_grid, int threshold){
	
	g->allocate_points_to_grid(cloud, cloud_normals);
	g->remove_voxels(threshold, false);	

	set_kdtree_grid(g, kdtree_grid);

}

void set_resolution(pcl::PointXYZ & resolution, pcl::PointXYZ minPoint, pcl::PointXYZ maxPoint){
	
	double xdiff = maxPoint.x - minPoint.x;
	double ydiff = maxPoint.y - minPoint.y;
	double zdiff = maxPoint.z - minPoint.z;

	resolution.x = DIM_FRAC * xdiff; //					max( DIM_FRAC * xdiff, 5.0);
	resolution.y = DIM_FRAC * ydiff; //					max( DIM_FRAC * ydiff, 5.0);
	resolution.z = DIM_FRAC * zdiff; //					max( DIM_FRAC * zdiff, 5.0);
	
}							

vector<Block> voxel_list(grid *g){

	Block b;
	vector<Block> v_list;
	
	for(int i = 0; i < g->length; i++){
		for(int j = 0; j < g->width; j++){
			for(int k = 0; k < g->height; k++){
				if(g->data[i][j][k].used){
					
					b = Block(i, j, k, 1, 1, 1);
					v_list.push_back(b);
				}
			}
		}
	}

	return v_list;

}

#endif		

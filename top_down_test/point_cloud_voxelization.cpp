#include <iostream>
#include <cstdio>
#include <cmath>
#include <vector>

/* point cloud header fils */

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/octree/octree.h>


/* end of header files */

using namespace std;

int main (int argc, char** argv)
{
	if(argc < 2){
		cout<<"Usage: <executabel binary> <path to .ply file>\n";
		exit(0);
	}	
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);					// new point cloud variable
	
	if (pcl::io::loadPLYFile<pcl::PointXYZ> (argv[1], *cloud) == -1)      // loading the .ply file into cloud
	{
		PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
		return (-1);
	}
	
	cout << "Loaded "
         << cloud->width 
         << " data points from "<<argv[1]<<endl;
         
    float resolution = 0.005f; 															// Resolution of each voxel     
    
    pcl::octree::OctreePointCloudVoxelCentroid<pcl::PointXYZ> octree (resolution);			//create octree object
	octree.setInputCloud (cloud);															// set point cloud input
	octree.addPointsFromInputCloud ();														// adding points to octree
		
	pcl::octree::OctreePointCloud<pcl::PointXYZ>::AlignedPointTVector centroid_list;		// list that will hold the centroids of the voxels.
	
	pcl::PointCloud<pcl::PointXYZ> voxel_cloud;									// point_cloud to hold voxelized cloud
	
	bool voxel_success = false;													// set to TRUE on succesfully creating point cloud				
	
	
	if (octree.getVoxelCentroids(centroid_list)){
		
		cout<<"Centroid list found!!!\n";
		cout<<"No. of voxels creted... "<<centroid_list.size()<<endl<<endl;
		
		voxel_cloud.width = centroid_list.size();								// Initialising voxel cloud
		voxel_cloud.height = 1;
		voxel_cloud.is_dense = true;
		voxel_cloud.points.resize(voxel_cloud.width * voxel_cloud.height);
	
		for(int i = 0; i<centroid_list.size(); i++){
		
			voxel_cloud.points[i].x = centroid_list[i].x;
			voxel_cloud.points[i].y = centroid_list[i].y;
			voxel_cloud.points[i].z = centroid_list[i].z;			//assigning voxels co-or to cloud
		}	
		
			voxel_success = true;								// succesfully created voxel cloud
	}
	else{
		
		cout<<"Cannot extract centroids\n";	
		exit(0);
	}
	
	
	if(voxel_success)
		pcl::io::savePLYFileASCII("../voxelized_bunny.ply",voxel_cloud);				// writing to file...
    else
		cout<<"Cannot write file!!!"<<endl;
    
    return 0;

}	
	

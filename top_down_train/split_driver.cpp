
/* to voxelize point cloud */

#include "voxelization_grid.cpp"

#include "top_down_split.cpp"

#include <algorithm>

#include <fstream>
#include <pcl/features/normal_3d.h>
#include "point_res.cpp"

using namespace std;

vector<Block> split_driver (char* filename, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, pcl::PointCloud<pcl::Normal>::Ptr &cloud_normals, grid * &g, pcl::PointXYZ &min, pcl::PointXYZ &max, int &length, int &width, int &height, double &resolution_in, char * ft_dir_name)
{
	
	fstream fs;
	
	fs.open("/home/rahul/Desktop/mtp/codes/top_down_train/arguments.txt", fstream::in);
	
	//cerr<<"Enter GAIN_THRESHOLD: ";
	fs>>VOL_THRESHOLD;
	
	//cerr<<"Enter BLOCK_SIZE_THRESHOLD: ";
	fs>>BLOCK_SIZE_THRESHOLD;
	
	double rmse_3d_thresh; 
	fs>>rmse_3d_thresh;
	
	int no_steps;
	fs>>no_steps;


	fs.close();

	//cerr<<"Volume Threshold :"<<VOL_THRESHOLD<<" BLOCK_SIZE_THRESHOLD :"<<BLOCK_SIZE_THRESHOLD<<"\n";
	
	if (pcl::io::loadPLYFile<pcl::PointXYZRGB> (filename, *cloud) == -1)      			// loading the .ply file into cloud
	{
		PCL_ERROR ("Couldn't read .ply file\n");
		exit(0);
	}
	
	// cerr << "Loaded "
    //     << cloud->width 
    //     << " data points from "<<filename<<endl;
         
    compute_normal(cloud, cloud_normals);
	
	find_min_max(*cloud, min, max);										// finding the min max point of cloud, defined in voxelization_grid.cpp
	
	pcl::PointXYZ resolution;
	set_resolution(resolution, min, max); 
	
	
	
	g = setup_grid(resolution, min, max);							// Setup the grid, function defined in voxelization_grid.cpp	
	
	cerr<<"The grid dimensions are "<<g->length<<"  "<<g->width<<"  "<<g->height<<"\n";
	
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_grid;
	
	int threshold = 3;											// setting threshold defined in voxelization_grid.cpp
	
	voxelize(g, *cloud, *cloud_normals, kdtree_grid, threshold);										// voxelize the point cloud, and mark used voxels from the grid	
																		// based on threshold. defined in voxelization_grid.cpp
	resolution_in = g->resolution;
	
	Block b = Block(0, 0, 0, g->length, g->width, g->height);
	
	
	vector<Block> final_list;
	
	
	vector<Block> decom;
	vector<Block> tree(20000);
	
	for(int i = 0; i<20000; i++)
		tree[i] = Block(0,0,0,0,0,0);
	
	decom = top_split(g, b, rmse_3d_thresh, filename, ft_dir_name, kdtree_grid, tree);
	//decom = voxel_list(g);
		
	length = g->length;
	width = g->width;
	height = g->height;
	 
	final_list.insert(final_list.end(), decom.begin(), decom.end());
	
	// cout<<block_list.size()<<"\n";
	//for(int i = 0; i < final_list.size(); i++){
	
		//cout<<final_list[i].x<<"  "<<final_list[i].y<<"  "<<final_list[i].z<<"  "<<final_list[i].length<<"  "<<final_list[i].width<<"  "<<final_list[i].height<<"\n";
	//}
	
	
	//return tree;
	return final_list;
		
}

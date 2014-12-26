
/* to voxelize point cloud */

#include "voxelization_grid.cpp"

#include "top_down_split.cpp"

#include <algorithm>

#include <fstream>

using namespace std;

void load_training_data(char * tr_file_name, vector<Training_block> &tr_blk_list){
	
	fstream fs;
	
	fs.open(tr_file_name, fstream::in);

	Training_block b;
	
	while(true){
		
		if( !(fs>>b.model_name) )
			break;
		fs>>b.model_number;
		
		fs>>b.asp_1;
		fs>>b.asp_2;
		fs>>b.asp_3;
				
		fs>>b.is_splited;
		
		if(b.is_splited > 0){
			fs>>b.x_1>>b.y_1>>b.z_1;
			fs>>b.l_1>>b.w_1>>b.h_1;
			
			fs>>b.x_2>>b.y_2>>b.z_2;
			fs>>b.l_2>>b.w_2>>b.h_2;
		}
		
		
		tr_blk_list.push_back(b);
	}

	cerr<<"Size of data :"<<tr_blk_list.size()<<"\n";

}

vector<Block> split_driver (char* filename, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, pcl::PointCloud<pcl::Normal>::Ptr &cloud_normals, pcl::PointXYZ &min, pcl::PointXYZ &max, int &length, int &width, int &height, double &resolution_in, char * ft_dir_name)
{
	
	/* Code for decomposition starts */
	
	fstream fs;
	
	fs.open("/home/rahulm/mtp/codes/top_down_test/arguments.txt", fstream::in);
	
	//cerr<<"Enter GAIN_THRESHOLD: ";
	fs>>VOL_THRESHOLD;
	
	//cerr<<"Enter BLOCK_SIZE_THRESHOLD: ";
	fs>>BLOCK_SIZE_THRESHOLD;
	
	double rmse_3d_thresh; 
	fs>>rmse_3d_thresh;
	
	int no_steps;
	fs>>no_steps;

	char tr_data_filename[100];
	fs>>tr_data_filename;
	
	
	fs.close();

	//cerr<<"Volume Threshold :"<<VOL_THRESHOLD<<" BLOCK_SIZE_THRESHOLD :"<<BLOCK_SIZE_THRESHOLD<<"\n";
	
	if (pcl::io::loadPLYFile<pcl::PointXYZ> (filename, *cloud) == -1)      			// loading the .ply file into cloud
	{
		PCL_ERROR ("Couldn't read .ply file\n");
		exit(0);
	}
	
	// cerr << "Loaded "
    //     << cloud->width 
    //     << " data points from "<<filename<<endl;
         
    int threshold = THRESHOLD;											// setting threshold defined in voxelization_grid.cpp

	compute_normal(cloud, cloud_normals);
	double cloud_res = computeCloudResolution(cloud);
	
	find_min_max(*cloud, min, max);										// finding the min max point of cloud, defined in voxelization_grid.cpp
	
	pcl::PointXYZ resolution;
	set_resolution(resolution, min, max); 
	
	// cout<<"Resolution "<<resolution.x<<"  "<<resolution.y<<"  "<<resolution.z<<"\n";
	
	grid *g = setup_grid(resolution, min, max);							// Setup the grid, function defined in voxelization_grid.cpp	
	
	cerr<<"The grid dimensions are "<<g->length<<"  "<<g->width<<"  "<<g->height<<"\n";
	
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_grid;
	
	
	voxelize(g, *cloud, *cloud_normals, kdtree_grid, threshold);										// voxelize the point cloud, and mark used voxels from the grid	
																		// based on threshold. defined in voxelization_grid.cpp
	resolution_in = g->resolution;
	
	vector<Training_block> tr_data_list;
	
	//load_training_data(tr_data_filename, tr_data_list);
	
	//pcl::KdTreeFLANN<pcl::PointXYZ> kdtree = load_kd_tree(tr_data_list);	
	
	Block b = Block(0, 0, 0, g->length, g->width, g->height);
	
	vector<Block> final_list;
	
	
	vector<Block> decom;
	
	decom = top_split(g, b, rmse_3d_thresh, ft_dir_name, kdtree, tr_data_list, cloud_res, kdtree_grid);
	//decom = voxel_list(g);
		
	length = g->length;
	width = g->width;
	height = g->height;
	 
	//final_list.insert(final_list.end(), decom.begin(), decom.end());
	
	// cout<<block_list.size()<<"\n";
	//for(int i = 0; i < final_list.size(); i++){
	
		//cout<<final_list[i].x<<"  "<<final_list[i].y<<"  "<<final_list[i].z<<"  "<<final_list[i].length<<"  "<<final_list[i].width<<"  "<<final_list[i].height<<"\n";
	//}
	
	
	
	return decom;

}

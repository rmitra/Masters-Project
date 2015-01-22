#include "split_driver.cpp"
#include "merge_block.cpp"

pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;	// Also required by draw_object.cpp

#include "./draw_object.cpp"

#include <pcl/point_types.h>
#include <pcl/features/shot.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/keypoints/iss_3d.h>

#include "point_res.cpp"

using namespace std;

int main(int argc, char **argv){

	if(argc < 3){
		cout<<"Usage: <executabel binary> <path to .ply file> <path to feature folder> <Scale for rendering>\n";
		exit(0);
	}
	
	grid *g;
	
	cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);	 // new point cloud variable
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals;
	
	pcl::PointXYZ min, max;		// To store min max
	
	int length, width, height;
	
	double resolution;
	
	vector<Block> block_list = split_driver(argv[1], cloud, cloud_normals, g, min, max, length, width, height, resolution, argv[2]);
	
	//cerr<<"No. blocks before merge "<<block_list.size()<<"\n";
	
	//merge_Block(block_list);
	
	//cerr<<"No. blocks after merge"<<block_list.size()<<"\n";
	
	grid *g_disp;
	
	double sub_factor = 3.0;	

	resolution = resolution / sub_factor;
	
	g_disp = new grid(g->length * 3, g->width * 3, g->height * 3, resolution, min, max);
	
	int threshold = 1;
	
	g_disp->allocate_points_to_grid_display(*cloud, *cloud_normals, resolution * sub_factor);
	g_disp->remove_voxels(threshold, false);
	
	initialise_parameters(g_disp, g, atoi(argv[3]), block_list, min, max);
	draw_main(argc, argv);	
	
	
	//for(int i = 0; i < block_list.size(); i++){
		//cout<<(double)block_list[i].height/(double)block_list[i].length<<", "<<(double)block_list[i].height/(double)block_list[i].width<<", "<<(double)block_list[i].width/(double)block_list[i].length<<"\n";
	//}
	
		
	return 0;


}	

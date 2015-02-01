// example call: ./box_decomposition ~/ply_data/test_whitehouse.ply 4

#include <pcl/point_types.h>
#include <pcl/features/shot.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/keypoints/iss_3d.h>

pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;	// Also required by draw_object.cpp

#include "grid.cpp"
#include "split_driver.cpp"
#include "merge_block.cpp"
#include "draw_object.cpp"

#include "point_res.cpp"

using namespace std;

int main(int argc, char **argv)
{
	if (argc < 2) {
		cout<<"Usage: <executable binary> <path to .ply file> <Scale for rendering>\n";
		exit(1);
	}
	
	grid *g;
	
	// new point cloud variable
	cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals;
	
	pcl::PointXYZ min, max;
	
	int length, width, height;
	double resolution;
	
	char directory_name[14] = "../../feature";
	vector<Block> block_list = split_driver(argv[1], cloud, cloud_normals, g, min, max, length, width, height, resolution, directory_name);

	trim_boxes(block_list);
	
	// grid used for displaying with OpenGL
	grid *g_disp;
	double sub_factor = 3.0;	
	resolution = resolution / sub_factor;
	g_disp = new grid(g->length * 3, g->width * 3, g->height * 3, resolution, min, max);
	
	int threshold = 1;
	
	g_disp->allocate_points_to_grid_display(*cloud, *cloud_normals, resolution * sub_factor);
	g_disp->remove_voxels(threshold, false);
	
	initialise_parameters(g_disp, g, atoi(argv[2]), block_list, min, max);
	draw_main(argc, argv);	
		
	return 0;
}	

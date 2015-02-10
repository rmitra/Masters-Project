/* to voxelize point cloud */

#include <algorithm>
#include <fstream>
#include <pcl/features/normal_3d.h>

#include "top_down_split.cpp"
#include "voxelization_grid.cpp"
#include "point_res.cpp"

using namespace std;

vector<Block> split_driver(char* filename, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, pcl::PointCloud<pcl::Normal>::Ptr &cloud_normals, grid * &g, pcl::PointXYZ &min, pcl::PointXYZ &max, int &length, int &width, int &height, double &resolution_in, char * ft_dir_name)
{
	double rmse_3d_thresh;

	// DIM_FRAC is voxel size
	fstream fs; fs.open("../arguments.txt", fstream::in);
	fs >> VOL_THRESHOLD >> BLOCK_SIZE_THRESHOLD >> DIM_FRAC >> rmse_3d_thresh;
	fs.close();

	// loading the .ply file into cloud
	if (pcl::io::loadPLYFile<pcl::PointXYZRGB> (filename, *cloud) == -1)
	{
		PCL_ERROR("Couldn't read .ply file\n");
		exit(0);
	}

    compute_normal(cloud, cloud_normals);

    // finding the min max point of cloud, defined in voxelization_grid.cpp
	find_min_max(*cloud, min, max);

	pcl::PointXYZ resolution;
	set_resolution(resolution, min, max);

	// Setup the grid, function defined in voxelization_grid.cpp
	g = setup_grid(resolution, min, max);

	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_grid;

	// setting threshold defined in voxelization_grid.cpp
	int threshold = 3;

	// voxelize the point cloud, and mark used voxels from the grid
	// based on threshold. defined in voxelization_grid.cpp
	voxelize(g, *cloud, *cloud_normals, kdtree_grid, threshold);

	resolution_in = g->resolution;

	Block b(0, 0, 0, g->length, g->width, g->height);

	vector<Block> decom;
	vector<Block> tree(20000);

	decom = top_split(g, b, rmse_3d_thresh, filename, ft_dir_name, kdtree_grid, tree);

	length = g->length;
	width = g->width;
	height = g->height;

	return decom;
}

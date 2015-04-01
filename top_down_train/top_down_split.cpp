#include <queue>
#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <vector>
#include <queue>
#include <stack>
#include <string>

#include "grid.hpp"
#include "extract_features.cpp"

using namespace std;

#define VOXEL_THRESHOLD 0.90

// set this to 1 (true) if using both the volume heuristic and the RMSE heuristic
#define USING_VOL_AND_RMSE 1

double VOL_THRESHOLD;
double BLOCK_SIZE_THRESHOLD;

int start;

#include "rmse.cpp"

// in the report, according to algorithm 4.2, we fill up the arrays in this function
// but in this implementation, the arrays have already been filled up by the calculate_gain_red function
double get_min_rmse_split(int begin, int end, double er_l[], double er_u[], double vol_l[], double vol_u[], int &er_cutIndex, double &less_er_vol, bool &inside_loop)
{
	bool is_lower = false;

	int temp_cut_lower = -1;
	int temp_cut_upper = -1;

	double max_rmse_jump = -1; // anything < 0 will do
	double prev_rmse = 10000; // any large number will do
	double min_rmse_lower_th = 100000.0; // any large number will do
	double min_rmse_upper_th = 100000.0; // any large number will do

	// first compute the minimum elements of er_l and er_u
	for (int i = begin + 4; i <= end - 4; i++) {
		if(er_l[i] < min_rmse_lower_th)
			min_rmse_lower_th = er_l[i];

		if(er_u[i] < min_rmse_upper_th)
			min_rmse_upper_th = er_u[i];
	}

	// for (int i = begin + 4; i <= end - 5; i++) {
	// 	cout << er_l[i] << "___" << er_u[i] << " ";
	// } cout << endl;

	// then find the i at which the jumps in er_l and -1*er_u are maximised
	// with the condition that er_l or er_u are within a threshold of the min_rmse
	for (int i = begin + 4; i <= end - 5; i++) {
		if (er_l[i+1] > er_l[i] && er_l[i] <= min_rmse_lower_th + 0.03) {
			if (er_l[i] < prev_rmse && er_l[i+1] - er_l[i] > max_rmse_jump - 0.03) {
				max_rmse_jump = er_l[i+1] - er_l[i];
				temp_cut_lower = i;
				is_lower = true;
				prev_rmse = er_l[i];
				inside_loop = true;
			}
		}

		if (er_u[i] > er_u[i+1] && er_u[i+1] <= min_rmse_upper_th + 0.03) {
			if (er_u[i+1] < prev_rmse && er_u[i] - er_u[i+1] > max_rmse_jump - 0.03) {
				max_rmse_jump = er_u[i] - er_u[i+1];
				temp_cut_upper = i;
				is_lower = false;
				prev_rmse = er_u[i+1];
				inside_loop = true;
			}
		}
	}

	// cout << "about to return from get_min_rmse_split, inside_loop " << inside_loop << endl;
	if (inside_loop) {
		er_cutIndex = is_lower ? temp_cut_lower : temp_cut_upper;
		if (er_l[er_cutIndex] < er_u[er_cutIndex]) {
			return er_l[er_cutIndex];
		}
		else {
			return er_u[er_cutIndex];
		}
	}
	else {
		less_er_vol = -1.0;
		return -1.0;
	}
}

//calculate gain in one direction
double calculate_gain_red(grid * g, int plane, int x, int y, int z, int length, int width, int height,  int &cutIndex, bool &is_red, double &less_er_vol, pcl::KdTreeFLANN<pcl::PointXYZ> &kdtree_grid)
{
	int min_x_u, max_x_u;
	int min_y_u, max_y_u;
	int min_z_u, max_z_u;

	int min_x_l, max_x_l;
	int min_y_l, max_y_l;
	int min_z_l, max_z_l;

	double er_u[500], vol_u[500];
	double er_l[500], vol_l[500];

	double min_red = 2;			 // Anything Greater than 1 //
	is_red = false;

	bool inside_loop = true;

	int red_cutIndex = -1;
	int er_cutIndex = -1;

	int begin = start;
	int end;

	switch (plane) {
	case 0:
		end = height - 1 - begin;
		break;
	case 1:
		end = width - 1 - begin;
		break;
	case 2:
		end = length - 1 - begin;
		break;
	}

	Block b;

	// The calls to cal_min_max_3D are done for the volume heuristic
	// The calls to get_rmse_3D are for the RMSE heuristic, in case the volume heuristic fails
	for(int i = begin; i <= end; i++) {
		if(plane == 0)
		{
			// For lower part //
			cal_min_max_3D(g, x, y, z, x+length, y+width, z+i+1, plane, min_x_l, min_y_l, min_z_l, max_x_l, max_y_l, max_z_l);
			b = Block(min_x_l, min_y_l, min_z_l, (max_x_l-min_x_l+1), (max_y_l-min_y_l+1), (max_z_l-min_z_l+1));
			er_l[i] = get_rmse_3D(g, b, kdtree_grid, false);

			// For upper part //
			cal_min_max_3D(g, x, y, z+i+1, x+length, y+width, z+height, plane, min_x_u, min_y_u, min_z_u, max_x_u, max_y_u, max_z_u);
			b = Block(min_x_u, min_y_u, min_z_u, (max_x_u-min_x_u+1), (max_y_u-min_y_u+1), (max_z_u-min_z_u+1));
			er_u[i+1] = get_rmse_3D(g, b, kdtree_grid, false);

		}
		else if(plane == 1)
		{
			// For lower part //
			cal_min_max_3D(g, x, y, z, x+length, y+i+1, z+height, plane, min_x_l, min_y_l, min_z_l, max_x_l, max_y_l, max_z_l);
			b = Block(min_x_l, min_y_l, min_z_l, (max_x_l-min_x_l+1), (max_y_l-min_y_l+1), (max_z_l-min_z_l+1));
			er_l[i] = get_rmse_3D(g, b, kdtree_grid, false);

			// For upper part //
			cal_min_max_3D(g, x, y+i+1, z, x+length, y+width, z+height, plane, min_x_u, min_y_u, min_z_u, max_x_u, max_y_u, max_z_u);
			b = Block(min_x_u, min_y_u, min_z_u, (max_x_u-min_x_u+1), (max_y_u-min_y_u+1), (max_z_u-min_z_u+1));
			er_u[i+1] = get_rmse_3D(g, b, kdtree_grid, false);
		}
		else
		{
			// For lower part //
			cal_min_max_3D(g, x, y, z, x+i+1, y+width, z+height, plane, min_x_l, min_y_l, min_z_l, max_x_l, max_y_l, max_z_l);
			b = Block(min_x_l, min_y_l, min_z_l, (max_x_l-min_x_l+1), (max_y_l-min_y_l+1), (max_z_l-min_z_l+1));
			er_l[i] = get_rmse_3D(g, b, kdtree_grid, false);

			// For upper part //
			cal_min_max_3D(g, x+i+1, y, z, x+length, y+width, z+height, plane, min_x_u, min_y_u, min_z_u, max_x_u, max_y_u, max_z_u);
			b = Block(min_x_u, min_y_u, min_z_u, (max_x_u-min_x_u+1), (max_y_u-min_y_u+1), (max_z_u-min_z_u+1));
			er_u[i+1] = get_rmse_3D(g, b, kdtree_grid, false);
		}

#if USING_VOL_AND_RMSE
		// volume heuristic for splitting - compute the ratio of net volume after splitting to volume before splitting
		double u_vol = (max_x_u - min_x_u + 1) * (max_y_u - min_y_u + 1) * (max_z_u - min_z_u + 1);
		double l_vol = (max_x_l - min_x_l + 1) * (max_y_l - min_y_l + 1) * (max_z_l - min_z_l + 1);

		vol_l[i] = l_vol;
		vol_u[i] = u_vol;

		double red = (u_vol + l_vol) / (double)(length * width * height);

		if (min_red > red) {
			min_red = red;
			red_cutIndex = i;
		}
#endif
	}

	// for (int i = begin; i <= end; i++) {
	// 	cout << er_l[i] << " " << er_u[i] << endl;
	// }

#if USING_VOL_AND_RMSE
	// We use the value of min_red to choose between using the volume heuristic and the RMSE heuristic
	double gain;
	if (min_red >= 0.0 && min_red < VOL_THRESHOLD)
	{
		// Use the volume heuristic directly
		gain = min_red;
		is_red = true;
		less_er_vol = -1.0;
	}
	else
	{
		// Volume heuristic not good enough, use RMSE
		inside_loop = false;
		gain = get_min_rmse_split(0, end + begin, er_l, er_u, vol_l, vol_u, er_cutIndex, less_er_vol, inside_loop);
		is_red = false;
	}

	if (inside_loop) {
		if(is_red)
			cutIndex = red_cutIndex;
		else
			cutIndex = er_cutIndex;
		return gain;
	}
	else
		return -1.0;
#else
	// Directly using only the RMSE
	inside_loop = false;
	double gain = get_min_rmse_split(0, end+begin, er_l, er_u, vol_l, vol_u, er_cutIndex, less_er_vol, inside_loop);
	cutIndex = er_cutIndex;
	is_red = true;
	// cout << "plane " << plane << " cutIndex " << cutIndex << " gain " << gain << endl;
	return (inside_loop ? gain : -1);
#endif
}

// plane has values 0 for xz, 1 for xy and 2 for yz  //
double find_red(grid *g, Block &parent, int plane, int &cutIndex, bool &is_red, double &less_er_vol, pcl::KdTreeFLANN<pcl::PointXYZ> &kdtree_grid)
{
	// calculating gain and dir of split for the plane
	double gain_red = calculate_gain_red(g, plane, parent.x, parent.y, parent.z, parent.length, parent.width, parent.height, cutIndex, is_red, less_er_vol, kdtree_grid);
	return (gain_red >= 0 ? gain_red : -1);
}

bool partition(grid *g, Block &parent, Block &ch1, Block &ch2, int &plane, int &final_cutIndex, pcl::KdTreeFLANN<pcl::PointXYZ> &kdtree_grid)
{
	int cutIndex_0, cutIndex_1, cutIndex_2;

	double min_gain_red = 100000;	// Any Big Number //
	double gain_red_0, gain_red_1, gain_red_2;

	bool is_red_0, is_red_1, is_red_2;
	double vol_0, vol_1, vol_2;

	gain_red_0 = find_red(g, parent, 0, cutIndex_0, is_red_0, vol_0, kdtree_grid); // Finding gain in direction 0
	gain_red_1 = find_red(g, parent, 1, cutIndex_1, is_red_1, vol_1, kdtree_grid); // Finding gain in direction 1
	gain_red_2 = find_red(g, parent, 2, cutIndex_2, is_red_2, vol_2, kdtree_grid); // Finding gain in direction 2

	if (gain_red_0 >= 0.0 || gain_red_1 >= 0.0 || gain_red_2 >= 0.0) {
		if (!is_red_0 && !is_red_1 && !is_red_2) {
			double min_err = gain_red_0;

			if (gain_red_1 < min_err)
				min_err = gain_red_1;

			if (gain_red_2 < min_err)
				min_err = gain_red_2;

			double allowance = 0.001;

			if (min_err < 0.0)
				return false;

			double max_vol = -1.0;

			if (gain_red_0 < min_err + allowance) {// && vol_0 > max_vol){
				plane = 0;
				final_cutIndex = cutIndex_0;
				max_vol = vol_0;
			}

			if (gain_red_1 < min_err + allowance) {// && vol_1 > max_vol){
				plane = 1;
				final_cutIndex = cutIndex_1;
				max_vol = vol_1;
			}

			if (gain_red_2 < min_err + allowance) {// && vol_2 > max_vol){
				plane = 2;
				final_cutIndex = cutIndex_2;
				max_vol = vol_2;
			}
		}
		else {
			if (is_red_0 == true) {
				min_gain_red = gain_red_0, plane = 0, final_cutIndex = cutIndex_0;
			}

			if (is_red_1 == true) {
				if(gain_red_1 < min_gain_red){
					min_gain_red = gain_red_1, plane = 1, final_cutIndex = cutIndex_1;
				}
			}
			if (is_red_2 == true) {
				if(gain_red_2 < min_gain_red){
					min_gain_red = gain_red_2, plane = 2, final_cutIndex = cutIndex_2;
				}
			}
		}
	}
	else {
		return false;
	}

	int min_x_l, min_y_l, min_z_l, max_x_l, max_y_l, max_z_l;
	int min_x_u, min_y_u, min_z_u, max_x_u, max_y_u, max_z_u;

	if (plane == 0) {
		cal_min_max_3D( g, parent.x, parent.y, parent.z, parent.x + parent.length, parent.y + parent.width, parent.z + final_cutIndex + 1, plane, min_x_l, min_y_l, min_z_l, max_x_l, max_y_l, max_z_l);
		cal_min_max_3D( g, parent.x, parent.y, parent.z + final_cutIndex + 1 , parent.x + parent.length, parent.y + parent.width, parent.z + parent.height, plane, min_x_u, min_y_u, min_z_u, max_x_u, max_y_u, max_z_u);
	}

	if (plane == 1) {
		cal_min_max_3D( g, parent.x, parent.y, parent.z, parent.x + parent.length, parent.y + final_cutIndex + 1, parent.z + parent.height, plane, min_x_l, min_y_l, min_z_l, max_x_l, max_y_l, max_z_l);
		cal_min_max_3D( g, parent.x, parent.y + final_cutIndex + 1, parent.z, parent.x + parent.length, parent.y + parent.width, parent.z + parent.height, plane, min_x_u, min_y_u, min_z_u, max_x_u, max_y_u, max_z_u);
	}

	if (plane == 2) {
		cal_min_max_3D( g, parent.x, parent.y, parent.z, parent.x + final_cutIndex + 1, parent.y + parent.width, parent.z + parent.height, plane, min_x_l, min_y_l, min_z_l, max_x_l, max_y_l, max_z_l);
		cal_min_max_3D( g, parent.x + final_cutIndex + 1, parent.y, parent.z, parent.x + parent.length, parent.y + parent.width, parent.z + parent.height, plane, min_x_u, min_y_u, min_z_u, max_x_u, max_y_u, max_z_u);
	}

	ch1.x = min_x_l, ch1.y = min_y_l, ch1.z = min_z_l;
	ch1.length = (max_x_l - min_x_l + 1), ch1.width = (max_y_l - min_y_l + 1), ch1.height = (max_z_l - min_z_l + 1);

	ch2.x = min_x_u, ch2.y = min_y_u, ch2.z = min_z_u;
	ch2.length = (max_x_u - min_x_u + 1), ch2.width = (max_y_u - min_y_u + 1), ch2.height = (max_z_u - min_z_u + 1);

	return true;
}


vector<Block> top_split(grid *g, Block init, double rmse_3d_thresh, char *filename, char * ft_dir_name, pcl::KdTreeFLANN<pcl::PointXYZ> &kdtree_grid, vector<Block> &tree)
{
	stack<Block> s;
	vector<Block> BlockList;
	stack<int> parent;

	s.push(init);

	bool limit_check;

	int i = 1;

	int min_dim = g->length;
	if (min_dim > g->width) min_dim = g->width;
	if (min_dim > g->height) min_dim = g->height;

	start = max(1, (int)( ( BLOCK_SIZE_THRESHOLD * min_dim ) / 2.0 ));

	string model_name = extract_name(filename);

	int plane, cut_index;
	int loop_count = 0;
	int tree_itr = 1;

	parent.push(tree_itr);

	// while (!s.empty() && loop_count < 20) {
	while (!s.empty()) {
		loop_count++;

		Block top = s.top();
		s.pop();

		tree_itr = parent.top();
		parent.pop();

		tree[tree_itr].x = top.x, tree[tree_itr].y = top.y, tree[tree_itr].z = top.z;
		tree[tree_itr].length = top.length, tree[tree_itr].width = top.width, tree[tree_itr].height = top.height;

		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
		pcl::PointCloud<pcl::Normal>::Ptr cloud_normals;

		double rmse_3d = get_rmse_3D(g, top, kdtree_grid, true);
		Block part1, part2;

		if (rmse_3d > rmse_3d_thresh) {
			bool val = partition(g, top, part1, part2, plane, cut_index, kdtree_grid);
			// cout << "cut index " << cut_index << endl;
			if (val) {
				s.push(part1);
				parent.push(2 * tree_itr);
				s.push(part2);
				parent.push(2 * tree_itr + 1);
			}
			else{
				BlockList.push_back(top);
			}
		}
		else {
			BlockList.push_back(top);
		}
	}
	return BlockList;
}
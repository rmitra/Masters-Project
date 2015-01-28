#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <string>

#include <pcl/kdtree/impl/kdtree_flann.hpp>

using namespace std;

/*bool found_cover(grid *g, Block b, int i, int j, int k, int plane){

	int count = 0;

	if(plane == 0){

		if(g->data[i][j][k].used)
			count++;
		if(k+1 < b.z + b.height && g->data[i][j][k+1].used)
			count++;
		if(k-1 >= b.z && g->data[i][j][k-1].used)
			count++;
		if(j+1 < b.y + b.width && g->data[i][j+1][k].used)
			count++;
		if(j+1 < b.y + b.width && k+1 < b.z + b.height && g->data[i][j+1][k+1].used)
			count++;
		if(j+1 < b.y + b.width && k-1 >= b.z && g->data[i][j+1][k-1].used)
			count++;
		if(j-1 >= b.y && g->data[i][j-1][k].used)
			count++;
		if(j-1 >= b.y && k+1 < b.z + b.height && g->data[i][j-1][k+1].used)
			count++;
		if(j-1 >= b.y && k-1 >= b.z && g->data[i][j-1][k-1].used)
			count++;
	}

	if(plane == 1){

		if(g->data[i][j][k].used)
			count++;
		if(k+1 < b.z + b.height && g->data[i][j][k+1].used)
			count++;
		if(k-1 >= b.z && g->data[i][j][k-1].used)
			count++;
		if(i+1 < b.x + b.length && g->data[i+1][j][k].used)
			count++;
		if(i+1 < b.x + b.length && k+1 < b.z + b.height && g->data[i+1][j][k+1].used)
			count++;
		if(i+1 < b.x + b.length && k-1 >= b.z && g->data[i+1][j][k-1].used)
			count++;
		if(i-1 >= b.x && g->data[i-1][j][k].used)
			count++;
		if(i-1 >= b.x && k+1 < b.z + b.height && g->data[i-1][j][k+1].used)
			count++;
		if(i-1 >= b.x && k-1 >= b.z && g->data[i-1][j][k-1].used)
			count++;
	}

	if(plane == 2){

		if(g->data[i][j][k].used)
			count++;
		if(j+1 < b.y + b.width && g->data[i][j+1][k].used)
			count++;
		if(j-1 >= b.y && g->data[i][j-1][k].used)
			count++;
		if(i+1 < b.x + b.length && g->data[i+1][j][k].used)
			count++;
		if(i+1 < b.x + b.length && j+1 < b.y + b.width && g->data[i+1][j+1][k].used)
			count++;
		if(i+1 < b.x + b.length && j-1 >= b.y && g->data[i+1][j-1][k].used)
			count++;
		if(i-1 >= b.x && g->data[i-1][j][k].used)
			count++;
		if(i-1 >= b.x && j+1 < b.y + b.width && g->data[i-1][j+1][k].used)
			count++;
		if(i-1 >= b.x && j-1 >= b.y && g->data[i-1][j-1][k].used)
			count++;
	}

	if(count >= 1)
		return true;
	else
		false;

}*/

bool is_occluded(pcl::KdTreeFLANN<pcl::PointXYZ> &kdtree_grid, int i, int j, int k)
{
	pcl::PointXYZ q_point;

	q_point.x = (double)i;
	q_point.y = (double)j;
	q_point.z = (double)k;

	vector<int> pointIdxRadiusSearch;
	vector<float> pointRadiusSquaredDistance;

	int found_instances = kdtree_grid.radiusSearch(q_point, 1.8, pointIdxRadiusSearch, pointRadiusSquaredDistance);

	return (found_instances < 5);
}


double get_rmse_3D(grid * g, Block &b, pcl::KdTreeFLANN<pcl::PointXYZ> &kdtree_grid, bool print_info)
{
	double sum_error = 0.0;

	int N = 0;

	for (int i = b.x; i < b.x + b.length; i++) {
		for (int j = b.y; j < b.y + b.width; j++) {
			for (int k = b.z; k < b.z + b.height; k++) {
				if (g->data[i][j][k].used) {
					bool is_boundary = false;

					int nearest_plane = 0;
					double near_distance = 1000000;

					//For side 1 //

					if(i - b.x < near_distance){

						//bool found = false;
						//for(int l = i-1; l >= b.x; l--){
						//	if(g->data[l][j][k].used)
						//		found = true;
						//}

						//if(!found){
							nearest_plane = 1;
							near_distance = i - b.x;
							is_boundary = true;
						//}
					}

					//For side 2 //

					if(b.x + b.length - 1 - i < near_distance){

					   // bool found = false;
						//for(int l = i+1; l < b.x + b.length; l++){
						//	 if(g->data[l][j][k].used)
						//	 	found = true;
						//}

						//if(!found){
							nearest_plane = 2;
							near_distance = b.x + b.length - 1 - i;
							is_boundary = true;
						//}
					}

					//For side 3 //

					if (j - b.y < near_distance) {

						//bool found = false;
						//for(int l = j-1; l >= b.y; l--){
						//	if(g->data[i][l][k].used)
						//		found = true;
						//}

						//if(!found){
							nearest_plane = 3;
							near_distance = j - b.y;
							is_boundary = true;
						//}
					}

					//For side 4 //

					if (b.y + b.width - 1 - j < near_distance) {

						//bool found = false;
						//for(int l = j+1; l < b.y + b.width; l++){
						//	 if(g->data[i][l][k].used)
						//	 	found = true;
					//	}

						//if(!found){
							nearest_plane = 4;
							near_distance = b.y + b.width - 1 - j;
							is_boundary = true;
					//	}
					}

					//For side 5 //

					if (k - b.z < near_distance) {

						//bool found = false;
						//for(int l = k-1; l >= b.z; l--){
						//	 if(g->data[i][j][l].used)
						//	 	found = true;
						//}

						//if(!found){
							nearest_plane = 5;
							near_distance = k - b.z;
							is_boundary = true;
					//	}
					}

					//For side 6 //

					if (b.z + b.height - 1 - k < near_distance) {

						//bool found = false;
						//for(int l = k+1; l < b.z + b.height; l++){
						//	 if(g->data[i][j][l].used)
						//	 	found = true;
						//}

						//if(!found){
							nearest_plane = 6;
							near_distance = b.z + b.height - 1 - k;
							is_boundary = true;
						//}
					}

					if (near_distance - 1 > 0) {
						near_distance = near_distance - 1;
					}
					else {
						near_distance = 0;
					}
					
					sum_error = sum_error + near_distance * near_distance;
					N++;
				}
			}
		}
	}

	double rmse;

	if (N == 0)
		rmse = 0;
	else {
		double mean_error = sum_error / (double)N;
		rmse = pow(mean_error, 0.5);
	}

	return rmse;
}

/*
void cal_min_max_3D(grid * g, int start_x, int start_y, int start_z, int stop_x, int stop_y, int stop_z, int &min_x, int &min_y, int &min_z, int &max_x, int &max_y, int &max_z){

	min_x = stop_x, min_y = stop_y, min_z = stop_z;
	max_x = start_x-1, max_y = start_y-1, max_z = start_z-1;

	bool found_data = false;

	for(int i = start_x; i<stop_x; i++){
		for(int j = start_y; j<stop_y; j++){
			for(int k = start_z; k<stop_z; k++){

				if(g->data[i][j][k].used == true)
				{
					found_data = true;

					if(min_x > i)
						min_x = i;

					if(min_y > j)
						min_y = j;

					if(min_z > k)
						min_z = k;

					if(max_x < i)
						max_x = i;

					if(max_y < j)
						max_y = j;

					if(max_z < k)
						max_z = k;

				}

			}
		}
	}

	if(!found_data){
		max_x = 0, min_x = 1;
		max_y = 0, min_y = 1;
		max_z = 0, min_z = 1;
	}

}
*/

void cal_min_max_3D_upper(grid * g, int start_x, int start_y, int start_z, int stop_x, int stop_y, int stop_z, int plane, int &min_x, int &min_y, int &min_z, int &max_x, int &max_y, int &max_z)
{
	min_x = stop_x, min_y = stop_y, min_z = stop_z;
	max_x = start_x-1, max_y = start_y-1, max_z = start_z-1;

	/*
	if(plane == 0)
		min_z = start_z;
	else if(plane == 1)
		min_y = start_y;
	else
		min_x = start_x;
	*/

	bool found_data = false;

	for(int i = start_x; i<stop_x; i++){
		for(int j = start_y; j<stop_y; j++){
			for(int k = start_z; k<stop_z; k++){

				if(g->data[i][j][k].used == true)
				{
					found_data = true;

					if(min_x > i) // && plane != 2)
						min_x = i;

					if(min_y > j) // && plane != 1)
						min_y = j;

					if(min_z > k) // && plane != 0
						min_z = k;

					if(max_x < i)
						max_x = i;

					if(max_y < j)
						max_y = j;

					if(max_z < k)
						max_z = k;

				}

			}
		}
	}

	if(!found_data){
		max_x = 0, min_x = 1;
		max_y = 0, min_y = 1;
		max_z = 0, min_z = 1;
	}

}

void cal_min_max_3D_lower(grid * g, int start_x, int start_y, int start_z, int stop_x, int stop_y, int stop_z, int plane, int &min_x, int &min_y, int &min_z, int &max_x, int &max_y, int &max_z){

	min_x = stop_x, min_y = stop_y, min_z = stop_z;
	max_x = start_x-1, max_y = start_y-1, max_z = start_z-1;

	/*
	if(plane == 0)
		max_z = stop_z - 1;
	else if(plane == 1)
		max_y = stop_y - 1;
	else
		max_x = stop_x - 1;
	*/
	bool found_data = false;

	for(int i = start_x; i<stop_x; i++){
		for(int j = start_y; j<stop_y; j++){
			for(int k = start_z; k<stop_z; k++){

				if(g->data[i][j][k].used == true)
				{
					found_data = true;

					if(min_x > i)
						min_x = i;

					if(min_y > j)
						min_y = j;

					if(min_z > k)
						min_z = k;

					if( max_x < i ) // && plane != 2)
						max_x = i;

					if(max_y < j) // && plane != 1)
						max_y = j;

					if(max_z < k) // && plane != 0)
						max_z = k;

				}

			}
		}
	}

	if(!found_data){
		max_x = 0, min_x = 1;
		max_y = 0, min_y = 1;
		max_z = 0, min_z = 1;
	}

}



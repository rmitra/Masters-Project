#include <queue>
#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <vector>
#include <queue>
#include <stack>
#include <string>

#include "grid_datastructure.cpp"
#include "extract_features.cpp"

// #include "./gmm/gmm_main.cpp"

using namespace std;

#define VOXEL_THRESHOLD 0.90

double VOL_THRESHOLD;  
double BLOCK_SIZE_THRESHOLD;
 
int start;

#include "rmse.cpp"


double get_min_rmse_split(int begin, int end, double er_l[], double er_u[], double vol_l[], double vol_u[], int &er_cutIndex, double &less_er_vol, bool &inside_loop)
{	
	//double rmse_1_diff[500];
	
	bool is_lower = false;
	
	int temp_cut_lower = -1;
	int temp_cut_upper = -1;
	
	double max_rmse_jump = -1;
	double prev_rmse = 10000;
	double min_rmse_lower_th = 100000.0;
	double min_rmse_upper_th = 100000.0;
	
	for(int i = begin + 4; i <= end - 4; i++ ){
		
		if(er_l[i] < min_rmse_lower_th)
			min_rmse_lower_th = er_l[i];
		
		if(er_u[i] < min_rmse_upper_th)
			min_rmse_upper_th = er_u[i];	
		
		//cerr<<"index : "<<i<<" l_err: "<<er_l[i]<<" u_err: "<<er_u[i]<<"\n";
	
	}
	

	for(int i = begin + 4; i <= end - 5; i++ ){
		
		//cerr<<"index : "<<i<<" "<<er_l[i]<<"\n";
		//cerr<<"index : "<<i<<" "<<fabs(er_l[i] - er_l[i+1])<<"\n";
		
		//inside_loop = true;
		
		if( er_l[i+1] > er_l[i] && er_l[i] <= min_rmse_lower_th + 0.03){ 
			if( er_l[i] < prev_rmse && er_l[i+1] - er_l[i] > max_rmse_jump - 0.03){
				max_rmse_jump = er_l[i+1] - er_l[i];
				temp_cut_lower = i;
				is_lower = true;
				prev_rmse = er_l[i];
				
				inside_loop = true;
			}
		}
		/*else if(er_l[i] > er_l[i+1] && er_l[i+1] <= min_rmse_lower_th + 0.03){
		
			if(er_l[i] - er_l[i+1] > max_rmse_jump){
				max_rmse_jump = er_l[i] - er_l[i+1];
				temp_cut_lower = i;
				is_lower = true;
				
				inside_loop = true;
			}
		}*/
		
		if( er_u[i] > er_u[i+1] && er_u[i+1] <= min_rmse_upper_th + 0.03){ 
			
			if(er_u[i+1] < prev_rmse && er_u[i] - er_u[i+1] > max_rmse_jump - 0.03){
				max_rmse_jump = er_u[i] - er_u[i+1];
				temp_cut_upper = i;
				is_lower = false;
				prev_rmse = er_u[i+1];
				
				inside_loop = true;
			}
		}
		/*else if(er_u[j] > er_u[j-1] && er_u[j-1] <= min_rmse_upper_th + 0.03){
		
			if(er_u[j] - er_u[j-1] > max_rmse_jump){
				max_rmse_jump = er_u[j] - er_u[j-1];
				temp_cut_upper = j;
				is_lower = false;
				
				inside_loop = true;
			}
		}*/
		
		
		
		
		//rmse_1_diff[i] = fabs(er_l[i] - er_l[i+1]);
	}	

	/*
	double max_rmse_2_diff = -1;
	double rmse_2_diff;
	
	cerr<<"RMSE_2_Diff\n";
	
	for(int i = begin + 3; i <= end - 2; i++ ){
		
		//" vol_l: "<<vol_l[i]<<" vol_u :"<<vol_u[i]<<"\n";
		
		inside_loop = true;
		
		
		//if( fabs(er_l[i] - er_l[i+1]) > max_rmse_jump ){
		//	max_rmse_jump = fabs(er_l[i] - er_l[i+1]);
		//	temp_cut = i;
		//}
		
		
		rmse_2_diff = fabs(rmse_1_diff[i] - rmse_1_diff[i+1]);
		
		//cerr<<"index : "<<i<<" "<<rmse_2_diff<<"\n";
		
		if( rmse_2_diff > max_rmse_2_diff ){
			max_rmse_2_diff = rmse_2_diff;
			temp_cut = i;
		}
		
	}
	*/		
	
	if(inside_loop){
				
		double less_err;
		
		if(is_lower)
			er_cutIndex = temp_cut_lower;
		else
			er_cutIndex = temp_cut_upper;
		
		
		if(er_l[er_cutIndex] < er_u[er_cutIndex]){
			less_err = er_l[er_cutIndex];
			
		}
		else{
			less_err = er_u[er_cutIndex];
			//err = er_u[er_cutIndex];
		}	
		cerr<<"cut_index :"<<er_cutIndex<<" less_vol :"<<less_er_vol<<"\n";
		return less_err;
		
	}
	else{
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
	
	bool inside_loop = false;
	
	int red_cutIndex = -1;
	int er_cutIndex = -1; 
		
	int end, begin;
	
	if(plane == 0) {
		begin = start;
		end = height - 1 - begin;
	}
	else if(plane == 1) {
		begin = start;
		end = width - 1 - begin;
	}
	else {
		begin = start;
		end = length - 1 - begin;
	}
	
	cerr << "For plane: " << plane << "\n";
	cerr << "Begin : " << begin << " End : " << end<<"\n";
	
	Block b;
	
	for(int i = begin; i <= end; i++) {
			
		inside_loop = true;
				
		if(plane == 0){
		
			// For lower part //
			cal_min_max_3D_lower(g, x, y, z, x+length, y+width, z+i+1, plane, min_x_l, min_y_l, min_z_l, max_x_l, max_y_l, max_z_l);
		
			b = Block(min_x_l, min_y_l, min_z_l, (max_x_l-min_x_l+1), (max_y_l-min_y_l+1), (max_z_l-min_z_l+1));
			er_l[i] = get_rmse_3D(g, b, kdtree_grid, false);
		
			// For upper part //
			cal_min_max_3D_upper(g, x, y, z+i+1, x+length, y+width, z+height, plane, min_x_u, min_y_u, min_z_u, max_x_u, max_y_u, max_z_u);
		
			b = Block(min_x_u, min_y_u, min_z_u, (max_x_u-min_x_u+1), (max_y_u-min_y_u+1), (max_z_u-min_z_u+1));
			er_u[i+1] = get_rmse_3D(g, b, kdtree_grid, false);
		
		}
		else if(plane == 1){
			
			// For lower part //
			cal_min_max_3D_lower(g, x, y, z, x+length, y+i+1, z+height, plane, min_x_l, min_y_l, min_z_l, max_x_l, max_y_l, max_z_l);
		
			b = Block(min_x_l, min_y_l, min_z_l, (max_x_l-min_x_l+1), (max_y_l-min_y_l+1), (max_z_l-min_z_l+1));
			er_l[i] = get_rmse_3D(g, b, kdtree_grid, false);
			
			// For upper part //
			cal_min_max_3D_upper(g, x, y+i+1, z, x+length, y+width, z+height, plane, min_x_u, min_y_u, min_z_u, max_x_u, max_y_u, max_z_u);
		
			b = Block(min_x_u, min_y_u, min_z_u, (max_x_u-min_x_u+1), (max_y_u-min_y_u+1), (max_z_u-min_z_u+1));
			er_u[i+1] = get_rmse_3D(g, b, kdtree_grid, false);
		}
		else{
		
			// For lower part //
			cal_min_max_3D_lower(g, x, y, z, x+i+1, y+width, z+height, plane, min_x_l, min_y_l, min_z_l, max_x_l, max_y_l, max_z_l);
		
			b = Block(min_x_l, min_y_l, min_z_l, (max_x_l-min_x_l+1), (max_y_l-min_y_l+1), (max_z_l-min_z_l+1));
			er_l[i] = get_rmse_3D(g, b, kdtree_grid, false);
			
						
			// For upper part //
			cal_min_max_3D_upper(g, x+i+1, y, z, x+length, y+width, z+height, plane, min_x_u, min_y_u, min_z_u, max_x_u, max_y_u, max_z_u);
		
			b = Block(min_x_u, min_y_u, min_z_u, (max_x_u-min_x_u+1), (max_y_u-min_y_u+1), (max_z_u-min_z_u+1));
			er_u[i+1] = get_rmse_3D(g, b, kdtree_grid, false);
		}
		
		double u_vol = (max_x_u - min_x_u + 1) * (max_y_u - min_y_u + 1) * (max_z_u - min_z_u + 1);
		double l_vol = (max_x_l - min_x_l + 1) * (max_y_l - min_y_l + 1) * (max_z_l - min_z_l + 1);
		
		vol_l[i] = l_vol;
		vol_u[i] = u_vol;
		
		double red = ( u_vol + l_vol ) / (double)(length * width * height);
		
		//cerr<<"index :"<<i<<" red :"<<red<<" u_vol :"<<u_vol<<" l_vol :"<<l_vol<<"\n";
		//cerr<<min_x_l<<" "<<min_y_l<<" "<<min_z_l<<"   "<<max_x_l<<" "<<max_y_l<<" "<<max_z_l<<"       "<<min_x_u<<" "<<min_y_u<<" "<<min_z_u<<"   "<<max_x_u<<" "<<max_y_u<<" "<<max_z_u<<"\n";		
		
		if(min_red > red){
			min_red = red;
			red_cutIndex = i;
			
		}		
	}
	
	double gain;
	
	cerr<<"Min red :"<<min_red<<" index : "<<red_cutIndex<<"\n";
	
	if( min_red >= 0.0 && min_red < VOL_THRESHOLD){
			gain = min_red;
			is_red = true;	
			less_er_vol = -1.0;
	}
	else{
		
		inside_loop = false;
		gain = get_min_rmse_split(0, end + begin, er_l, er_u, vol_l, vol_u, er_cutIndex, less_er_vol, inside_loop);
		cerr<<"MAX RMSE SECOND DERIVATIVE :"<<gain<<"\n";
		is_red = false;
	}
		
	if( inside_loop ){
		
		cerr<<"RMSE CUT INDEX :"<<er_cutIndex<<"\n";
		
		if(is_red)
			cutIndex = red_cutIndex;
		else 
			cutIndex = er_cutIndex;
				
		return gain;
	}	
	else
		return -1.0;
	
}


double find_red(grid *g, Block &parent , int plane, int &cutIndex, bool &is_red, double &less_er_vol, pcl::KdTreeFLANN<pcl::PointXYZ> &kdtree_grid){ 		// plane has values 0 for xz, 1 for xy and 2 for yz  //
																	
	// For XY Plane //
	
	if(plane == 0){
	
		double gain_red = calculate_gain_red(g, plane, parent.x, parent.y, parent.z, parent.length, parent.width, parent.height, cutIndex, is_red, less_er_vol, kdtree_grid);		// calculating gain and dir of split for XZ plane
	
		cerr<<"Final gain of plane XY  "<<gain_red<<"\n";
		if(gain_red >= 0.0){
		
			return gain_red;
		}	
		else
			return -1.0;
	}
	
	// For XZ Plane //
	
	if(plane == 1){
	
		double gain_red = calculate_gain_red(g, plane, parent.x, parent.y, parent.z, parent.length, parent.width, parent.height, cutIndex, is_red, less_er_vol, kdtree_grid);		// calculating gain and dir of split for XZ plane
	
		cerr<<"Final gain of plane XZ  "<<gain_red<<"\n";
		if( gain_red >= 0.0 ){
		
			return gain_red;
		}
		else
			return -1.0;
	}	
	
	// For YZ Plane //
	
	if(plane == 2){
	
		double gain_red = calculate_gain_red(g, plane, parent.x, parent.y, parent.z, parent.length, parent.width, parent.height, cutIndex, is_red, less_er_vol, kdtree_grid);		// calculating gain and dir of split for XZ plane
	
		cerr<<"Final gain of plane YZ  "<<gain_red<<"\n";
		if( gain_red >= 0.0 ){
		
			return gain_red;
		}
		else
			return -1.0;
	}
}		

bool partition(grid *g, Block &parent, Block &ch1, Block &ch2, int &plane, int &final_cutIndex, pcl::KdTreeFLANN<pcl::PointXYZ> &kdtree_grid){
	
	int cutIndex_0, cutIndex_1, cutIndex_2;
	// int final_cutIndex;
	
	double min_gain_red = 100000;	// Any Big Number //
	double gain_red_0, gain_red_1, gain_red_2;
	// int plane;
	
	bool is_red_0, is_red_1, is_red_2;
	double vol_0, vol_1, vol_2;
	
	gain_red_0 = find_red(g, parent, 0, cutIndex_0, is_red_0, vol_0, kdtree_grid);							// Finding gain in direction 0
	gain_red_1 = find_red(g, parent, 1, cutIndex_1, is_red_1, vol_1, kdtree_grid);							// Finding gain in direction 1
	gain_red_2 = find_red(g, parent, 2, cutIndex_2, is_red_2, vol_2, kdtree_grid);							// Finding gain in direction 2
	
	if(gain_red_0 >= 0.0 || gain_red_1 >= 0.0 || gain_red_2 >= 0.0){
		if(is_red_0 != true && is_red_1 != true && is_red_2 != true){
		
			double min_err = gain_red_0;
					
			if(gain_red_1 < min_err)
				min_err = gain_red_1;
				
			if(gain_red_2 < min_err)
				min_err = gain_red_2;
		
		
			double allowance = 0.001;
		
			if(min_err < 0.0 )
				return false;
			
			double max_vol = -1.0;
			
			if(gain_red_0 < min_err + allowance ){// && vol_0 > max_vol){
				plane = 0;
				final_cutIndex = cutIndex_0;	
			
				max_vol = vol_0;
			
			}
		
			if(gain_red_1 < min_err + allowance){// && vol_1 > max_vol){
				plane = 1;
				final_cutIndex = cutIndex_1;	
			
				max_vol = vol_1;
				
			}
		
			if(gain_red_2 < min_err + allowance){// && vol_2 > max_vol){
				plane = 2;
				final_cutIndex = cutIndex_2;	
			
				max_vol = vol_2;
			
			}
		
		}
		else{
			
			if(is_red_0 == true){
				min_gain_red = gain_red_0, plane = 0, final_cutIndex = cutIndex_0; 
			}
			
			if(is_red_1 == true){
				if(gain_red_1 < min_gain_red){
					min_gain_red = gain_red_1, plane = 1, final_cutIndex = cutIndex_1; 
				}
			
			}
		
			if(is_red_2 == true){
				if(gain_red_2 < min_gain_red){
					min_gain_red = gain_red_2, plane = 2, final_cutIndex = cutIndex_2; 
				}
			
			}
		
		}
			
	}
	else{
		return false;
	}
	
	cerr<<"Plane: "<<plane<<"  cutPoint: "<<final_cutIndex<<"\n";	

	int min_x_l, min_y_l, min_z_l, max_x_l, max_y_l, max_z_l;
	int min_x_u, min_y_u, min_z_u, max_x_u, max_y_u, max_z_u;
	
	if(plane == 0){
					
			cal_min_max_3D_lower( g, parent.x, parent.y, parent.z, parent.x + parent.length, parent.y + parent.width, parent.z + final_cutIndex + 1, plane, min_x_l, min_y_l, min_z_l, max_x_l, max_y_l, max_z_l);
			cal_min_max_3D_upper( g, parent.x, parent.y, parent.z + final_cutIndex + 1 , parent.x + parent.length, parent.y + parent.width, parent.z + parent.height, plane, min_x_u, min_y_u, min_z_u, max_x_u, max_y_u, max_z_u);
		
	}		

	if(plane == 1){
				
			cal_min_max_3D_lower( g, parent.x, parent.y, parent.z, parent.x + parent.length, parent.y + final_cutIndex + 1, parent.z + parent.height, plane, min_x_l, min_y_l, min_z_l, max_x_l, max_y_l, max_z_l);
			cal_min_max_3D_upper( g, parent.x, parent.y + final_cutIndex + 1, parent.z, parent.x + parent.length, parent.y + parent.width, parent.z + parent.height, plane, min_x_u, min_y_u, min_z_u, max_x_u, max_y_u, max_z_u);
			
	
	}
	
	if(plane == 2){
		
			cal_min_max_3D_lower( g, parent.x, parent.y, parent.z, parent.x + final_cutIndex + 1, parent.y + parent.width, parent.z + parent.height, plane, min_x_l, min_y_l, min_z_l, max_x_l, max_y_l, max_z_l);
			cal_min_max_3D_upper( g, parent.x + final_cutIndex + 1, parent.y, parent.z, parent.x + parent.length, parent.y + parent.width, parent.z + parent.height, plane, min_x_u, min_y_u, min_z_u, max_x_u, max_y_u, max_z_u);
	}
	
	
	// cerr<<"Lower Block: "<<min_x_l<<" "<<min_y_l<<"  "<<min_z_l<<"  "<<max_x_l<<"  "<<max_y_l<<"  "<<max_z_l<<"\n";
	// cerr<<"Upper Block: "<<min_x_u<<" "<<min_y_u<<"  "<<min_z_u<<"  "<<max_x_u<<"  "<<max_y_u<<"  "<<max_z_u<<"\n";

	//if(min_gain < GAIN_THRESHOLD){
		ch1.x = min_x_l, ch1.y = min_y_l, ch1.z = min_z_l; 	
		ch1.length = (max_x_l - min_x_l + 1), ch1.width = (max_y_l - min_y_l + 1), ch1.height = (max_z_l - min_z_l + 1);  
	
		ch2.x = min_x_u, ch2.y = min_y_u, ch2.z = min_z_u; 	
		ch2.length = (max_x_u - min_x_u + 1), ch2.width = (max_y_u - min_y_u + 1), ch2.height = (max_z_u - min_z_u + 1);
	
		return true;

}	
	

vector<Block> top_split(grid *g, Block init, double rmse_3d_thresh, char *filename, char * ft_dir_name, pcl::KdTreeFLANN<pcl::PointXYZ> &kdtree_grid, vector<Block> &tree){
	
	stack<Block> s;
	vector<Block> BlockList;
	stack<int> parent;
	
	s.push(init);
	
	
	bool limit_check;
	
	int i = 1;
	
	int min_dim = g->length; 
	
	if( min_dim > g->width)
		min_dim = g->width;
	
	if(min_dim > g->height)
		min_dim = g->height;
		
	start = max(1, (int)( ( BLOCK_SIZE_THRESHOLD * min_dim ) / 2.0 ));
	
	// cerr<<"BLOCK_T: "<<BLOCK_SIZE_THRESHOLD<<" MINDIM: "<<min_dim<<" Start :"<<start<<"\n";
		
	
	string model_name = extract_name(filename);
		
	int sub_model_count = 1;
		
	int plane, cut_index;
	
	int loop_count = 0;
	
	int tree_itr = 1;
	
	parent.push(tree_itr);
	
	while(!s.empty()){// && loop_count < 100){
		
		loop_count++;
		
		Block top = s.top();
		s.pop();
		
		tree_itr = parent.top();
		parent.pop();
		
		cerr<<"Tree Itr :"<<tree_itr<<"\n";
		
		tree[tree_itr].x = top.x, tree[tree_itr].y = top.y, tree[tree_itr].z = top.z;
		tree[tree_itr].length = top.length, tree[tree_itr].width = top.width, tree[tree_itr].height = top.height;
		
		
		cerr<<"For Box : "<<top.x<<" "<<top.y<<" "<<top.z<<" "<<top.x + top.length<<" "<<top.y + top.width<<" "<<top.z + top.height<<"\n";
		
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
		pcl::PointCloud<pcl::Normal>::Ptr cloud_normals;
		
		//get_cloud_from_grid(top, g, cloud, cloud_normals);		
		
		//cerr<<"Size of cloud of model "<<sub_model_count<<" is "<<cloud->size()<<"\n";
		
		//save_features(ft_dir_name, model_name, tree_itr, cloud, cloud_normals);
		
		double rmse_3d = get_rmse_3D(g, top, kdtree_grid, true);
				
		cerr<<"RMSE 3D: "<<rmse_3d<<"\n";
				
		//double size_frac_l = (double)( top.length ) / (double)(min_dim);
		//double size_frac_w = (double)( top.width ) / (double)(min_dim);
		//double size_frac_h = (double)( top.height ) / (double)(min_dim);
		Block part1, part2;
		
		if( rmse_3d > rmse_3d_thresh ){ // && size_frac_l > BLOCK_SIZE_THRESHOLD && size_frac_w > BLOCK_SIZE_THRESHOLD && size_frac_h > BLOCK_SIZE_THRESHOLD){
		
			
		
			bool val = partition(g, top, part1, part2, plane, cut_index, kdtree_grid); 
			
			cerr<<"Val: "<<val<<"\n";			 	


			if( val ){
		
					//print_aspect_ratio(model_name, top, tree_itr, true, plane, cut_index, part1, part2);
					
					s.push(part1);	
					
					//tree[2 * tree_itr].x = part1.x, tree[2 * tree_itr].y = part1.y, tree[2 * tree_itr].z = part1.z;
					//tree[2 * tree_itr].length = part1.length, tree[2 * tree_itr].width = part1.width, tree[2 * tree_itr].height = part1.height;
					parent.push(2 * tree_itr);
					
					s.push(part2);
					
					//tree[2 * tree_itr + 1].x = part2.x, tree[2 * tree_itr + 1].y = part2.y, tree[2 * tree_itr + 1].z = part2.z;
					//tree[2 * tree_itr + 1].length = part2.length, tree[2 * tree_itr + 1].width = part2.width, tree[2 * tree_itr + 1].height = part2.height;
					parent.push(2 * tree_itr + 1);
					
					cerr<<"Block 1: "<<part1.x<<" "<<part1.y<<" "<<part1.z<<" "<<part1.x + part1.length<<" "<<part1.y + part1.width<<" "<<part1.z + part1.height<<"\n";
					cerr<<"Block 2: "<<part2.x<<" "<<part2.y<<" "<<part2.z<<" "<<part2.x + part2.length<<" "<<part2.y + part2.width<<" "<<part2.z + part2.height<<"\n";
					cerr<<"\n";
			}	
			else{
				
				//print_aspect_ratio(model_name, top, tree_itr, false, -1, -1, part1, part2);
				
				BlockList.push_back(top);
			}	
		}
		else{ 
		
			//print_aspect_ratio(model_name, top, tree_itr, false, -1, -1, part1, part2);	
			
			BlockList.push_back(top);
			
		}
	
		sub_model_count++;
	}	
	
	cerr<<"BlockList size: "<<BlockList.size()<<"\n";
	
	return BlockList;

}				



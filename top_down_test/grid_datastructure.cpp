#include <iostream>
#include <cstdio>
#include <cmath>
#include <vector>

/* point cloud header fils */

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>

/* end of header files */

#ifndef GRID_HEADER
#define GRID_HEADER

#define EPS 0.001

using namespace std;

class grid_element {								// element for every grid node

		public:
		
		pcl::PointCloud<pcl::PointXYZ>::Ptr p_list;			// list of those points.	
		pcl::PointCloud<pcl::Normal>::Ptr n_list;			// list of normals.	
		
		//int rep[3];
		
		bool used;
		
		grid_element(){							// constructor
			
			p_list = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
			n_list = pcl::PointCloud<pcl::Normal>::Ptr(new pcl::PointCloud<pcl::Normal>);
			
			used = false; 
	    
		}

		/*void setRep(int x, int y, int z){
			rep[0] = x;
			rep[1] = y;
			rep[2] = z;
		}*/	

		

};

class grid {
	
	public:
		//	pcl::PointXYZ resolution;						// dimension of a cubical voxel
		double resolution;
		
		grid_element *** data;
		
		int length, width, height;					// dimension of the grid
		
		pcl::PointXYZ ref_point;					// here the min point is taken as the reference point
		
		/* Constructor taking resolution and min and max point of the point cloud
		   and determines the no. voxels in length, width and height.
		*/ 
		   		
		grid(pcl::PointXYZ res, pcl::PointXYZ minPoint, pcl::PointXYZ maxPoint){
			// resolution = res;
			
			if(res.x < res.y){
				if(res.x < res.z)
					resolution = res.x;
				else
					resolution = res.z;	
			}
			else{
				if(res.y < res.z)
					resolution = res.y;
				else
					resolution = res.z;		
			}
			
			
			// double xdiff_int = (int)( ((maxPoint.x - minPoint.x)/resolution) + 1.0f );
			// double xdiff = (maxPoint.x - minPoint.x)/resolution;   
			
			//if( xdiff_int - xdiff < 0.00001f)
			//	length = (int)( ((maxPoint.x - minPoint.x)/resolution) + 1.5f);
			//else
				length = (int)( ((maxPoint.x - minPoint.x)/resolution) + 1.0f);
						
			// double ydiff_int = (int)( ((maxPoint.y - minPoint.y)/resolution) + 1.0f );
			// double ydiff = (maxPoint.y - minPoint.y)/resolution;   
						
			//if( ydiff_int - ydiff < 0.00001f)
			//	width = (int)( ((maxPoint.y - minPoint.y)/resolution) + 1.5f);
			//else
				width = (int)( ((maxPoint.y - minPoint.y)/resolution) + 1.0f);
			
			// double zdiff_int = (int)( ((maxPoint.z - minPoint.z)/resolution) + 1.0f );
			// double zdiff = (maxPoint.z - minPoint.z)/resolution;   
						
			//if( zdiff_int - zdiff < 0.00001f)
			//	height = (int)( ((maxPoint.z - minPoint.z)/resolution) + 1.5f);
			//else
				height = (int)( ((maxPoint.z - minPoint.z)/resolution) + 1.0f);
			
			data  = new grid_element** [length];
			
			for(int i = 0; i < length; i++)
			{
				data[i] = new grid_element* [width];
				for( int j = 0; j < width; j++)
					data[i][j] = new grid_element[height];
			}		
				
		
			ref_point = minPoint;
		
		}
		
		void display_dimensions(){
		
			cout<<"Length: "<<length<<endl;
			cout<<"Width: "<<width<<endl;
			cout<<"Height: "<<height<<endl;
			
			cout<<"Resolution: "<<resolution<<"\n";      //<<"  "<<resolution.y<<"  "<<resolution.z<<endl<<endl;
			
			cout<<"Ref Point: x: "<<ref_point.x<<endl;
			cout<<"Ref Point: y: "<<ref_point.y<<endl;
			cout<<"Ref Point: z: "<<ref_point.z<<endl;
		
		}	

		
		/* function allocates a points of a cloud to a voxel in the grid
		 * Input point cloud
		 */ 
		
		void allocate_points_to_grid( pcl::PointCloud<pcl::PointXYZ> cloud, pcl::PointCloud<pcl::Normal> cloud_normals ){
			
			double xdiff, ydiff, zdiff;
			int xindex, yindex, zindex;
			
			
			for(int i = 0; i < cloud.points.size(); i++){
			
				xdiff = (cloud.points[i].x - ref_point.x)/resolution ;
				ydiff = (cloud.points[i].y - ref_point.y)/resolution ;
				zdiff = (cloud.points[i].z - ref_point.z)/resolution ;
			
				xindex = (int)(xdiff);	// + 1.0f); 	
				yindex = (int)(ydiff);	// + 1.0f);
				zindex = (int)(zdiff);	// + 1.0f);
			
				//if(xindex - xdiff > 0.00001f)
				//	xindex = xindex -1;
				
				//if(yindex - ydiff > 0.00001f)
				//	yindex = yindex - 1;
				
				//if(zindex - zdiff > 0.00001f)
				//	zindex = zindex - 1;	
				
				data[xindex][yindex][zindex].p_list->points.push_back(cloud.points[i]);
				data[xindex][yindex][zindex].n_list->points.push_back(cloud_normals.points[i]);
			}			
			
		
			return;
		
		}	

		void remove_voxels(int threshold){
			
			
			long voxel_used_count = 0;
			
			for(int i = 0; i < length; i++){
				for(int j = 0; j < width; j++){
					for(int k = 0; k < height; k++){
						
						
						if(data[i][j][k].p_list->points.size() >= threshold){
							data[i][j][k].used = true;
							voxel_used_count++;
						 }	
					}
				}
			}		

			// cout<<"Voxel used after thresholding: "<<voxel_used_count<<endl;
			
			return;
		}


};

class Block{
	public:
	int x;
	int y;
	int z;
	
	int length;
	int width;
	int height;
	
	Block(){
		x = y = z = 0;
		length = width = height = 1;
	}
	
	Block(const Block &b){
	
		x = b.x;
		y = b.y;
		z = b.z;
		
		length = b.length;
		width = b.width;
		height = b.height;
	}	
	
	
	Block(int x_i, int y_i, int z_i, int l_i, int w_i, int h_i ){
		
		x = x_i;
		y = y_i;
		z = z_i;
		
		length = l_i;
		width = w_i;
		height = h_i;
	
	}
	
};
		
class Training_block{

	public:
	double asp_1;
	double asp_2;
	double asp_3;
	
	int model_number;
	int is_splited;
	//double cut_frac;
	
	double x_1, y_1, z_1;
	double x_2, y_2, z_2;
	
	double l_1, w_1, h_1;
	double l_2, w_2, h_2;
	
	string model_name;
		
	Training_block(){
		asp_1 = asp_2 = asp_3 = 0.0;
		x_1 = y_1 = z_1 = l_1 = w_1 = h_1 = -1;
		x_2 = y_2 = z_2 = l_2 = w_2 = h_2 = -1;
		
		model_number = -1;
		is_splited = -1;
	}	

	Training_block(double _asp_1, double _asp_2, double _asp_3, int _model_number, double _x_1, double _y_1, double _z_1, double _l_1, double _w_1, double _h_1, double _x_2, double _y_2, double _z_2, double _l_2, double _w_2, double _h_2){
		asp_1 = _asp_1; 
		asp_2 = _asp_2;
		asp_3 = _asp_3;
		
		x_1 = _x_1;	
		y_1 = _y_1;
		z_1 = _z_1;
		
		l_1 = _l_1;	
		w_1 = _w_1;
		h_1 = _h_1;
	
		x_2 = _x_2;	
		y_2 = _y_2;
		z_2 = _z_2;
		
		l_2 = _l_2;	
		w_2 = _w_2;
		h_2 = _h_2;
	
		model_number = _model_number;
	
	}

	Training_block(const Training_block &b){
		
		asp_1 = b.asp_1;
		asp_2 = b.asp_2;
		asp_3 = b.asp_3;
		
		//plane = b.plane;
		//cut_frac = b.cut_frac;
		
		x_1 = b.x_1;
		y_1 = b.y_1;
		z_1 = b.z_1;
		
		l_1 = b.l_1;
		w_1 = b.w_1;
		h_1 = b.h_1;
		
		x_2 = b.x_2;
		y_2 = b.y_2;
		z_2 = b.z_2;
		
		l_2 = b.l_2;
		w_2 = b.w_2;
		h_2 = b.h_2;
		
		is_splited = b.is_splited;
		model_number = b.model_number;
		
		model_name = b.model_name;
	
	}
};

int search_block(string model_name, int model_number, vector<Training_block> &tr_data_list){

	for(int i = 0; i < tr_data_list.size(); i++){
		if( tr_data_list[i].model_name.compare(model_name) == 0 && tr_data_list[i].model_number == model_number)
			return i;
	}
	
	return -1;
}	




#endif

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
		
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_list;			// list of those points.	
		pcl::PointCloud<pcl::Normal>::Ptr n_list;			// list of normals.	
		double r,g,b;
		//int rep[3];
		
		bool used;
		
		grid_element(){							// constructor
			
			p_list = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
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
		
			//cerr<<"$$$$$$$"<<(int)( (maxPoint.z - minPoint.z)/resolution );
		
		}
		
		grid(double res, pcl::PointXYZ minPoint, pcl::PointXYZ maxPoint){
			
			resolution = res;
			
			length = (int)( ((maxPoint.x - minPoint.x)/resolution) + 1.0f);
			width = (int)( ((maxPoint.y - minPoint.y)/resolution) + 1.0f);
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
		
		grid(int _length, int _width, int _height, double res, pcl::PointXYZ minPoint, pcl::PointXYZ maxPoint){
			
			resolution = res;
			
			length = _length;
			width = _width;
			height = _height; 
			
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
		
		void allocate_points_to_grid( pcl::PointCloud<pcl::PointXYZRGB> cloud, pcl::PointCloud<pcl::Normal> cloud_normals ){
			
			double xdiff, ydiff, zdiff;
			int xindex, yindex, zindex;
			
			for(int i = 0; i < cloud.points.size(); i++){
			
				xdiff = (cloud.points[i].x - ref_point.x)/resolution ;
				ydiff = (cloud.points[i].y - ref_point.y)/resolution ;
				zdiff = (cloud.points[i].z - ref_point.z)/resolution ;
			
				xindex = (int)(xdiff);	// + 1.0f); 	
				yindex = (int)(ydiff);	// + 1.0f);
				zindex = (int)(zdiff);	// + 1.0f);
			
				/*if(zindex == 20)
					count_20++;
				else if(zindex == 19)
					count_19++;
				else if(zindex == 18)
					count_18++;		
				*/
				
				//if(yindex - ydiff > 0.00001f)
				//	yindex = yindex - 1;
				
				//if(zindex - zdiff > 0.00001f)
				//	zindex = zindex - 1;	
				
				
				
				
				
				data[xindex][yindex][zindex].p_list->points.push_back(cloud.points[i]);
				data[xindex][yindex][zindex].n_list->points.push_back(cloud_normals.points[i]);
			}			
			
			//cerr<<"Count 20: "<<count_20<<" "<<count_19<<" "<<count_18<<"\n";
		
			return;
		
		}	

		void remove_voxels(int threshold, bool disp){
			
			
			long voxel_used_count = 0;
			
			for(int i = 0; i < length; i++){
				for(int j = 0; j < width; j++){
					for(int k = 0; k < height; k++){
						
						
						if(data[i][j][k].p_list->points.size() >= threshold){
							
							if(disp)
								cerr<<"Inside!!!!!\n";
								
							data[i][j][k].used = true;
							voxel_used_count++;
						 
							data[i][j][k].r = data[i][j][k].g = data[i][j][k].b = 0.0;
							for(int l = 0; l < data[i][j][k].p_list->points.size(); l++){
								data[i][j][k].r = data[i][j][k].r + (data[i][j][k].p_list->points[l].r)/255.0;
								data[i][j][k].g = data[i][j][k].g + (data[i][j][k].p_list->points[l].g)/255.0;
								data[i][j][k].b = data[i][j][k].b + (data[i][j][k].p_list->points[l].b)/255.0;
							}
							
							data[i][j][k].r = data[i][j][k].r / data[i][j][k].p_list->points.size();
							data[i][j][k].g = data[i][j][k].g / data[i][j][k].p_list->points.size();
							data[i][j][k].b = data[i][j][k].b / data[i][j][k].p_list->points.size(); 
							
						}
						else{
							data[i][j][k].r = data[i][j][k].g = data[i][j][k].b = 0;
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
		

#endif

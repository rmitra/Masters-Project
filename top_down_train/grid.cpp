#include "grid.hpp"

/* Constructor taking resolution and min and max point of the point cloud
   and determines the no. voxels in length, width and height.
*/
grid::grid(pcl::PointXYZ res, pcl::PointXYZ minPoint, pcl::PointXYZ maxPoint){
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

	cerr << "Created Grid with dimensions " << length << " " << width << " " << height << ". ";
}

grid::grid(double res, pcl::PointXYZ minPoint, pcl::PointXYZ maxPoint)
{
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

	cerr << "Created Grid with dimensions " << length << " " << width << " " << height << ". ";

	ref_point = minPoint;
}

grid::grid(int _length, int _width, int _height, double res, pcl::PointXYZ minPoint, pcl::PointXYZ maxPoint)
{
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


	cerr << "Created Grid with dimensions " << length << " " << width << " " << height << ". ";
	ref_point = minPoint;
}

void grid::display_dimensions()
{
	cout<<"Length: "<<length<<endl;
	cout<<"Width: "<<width<<endl;
	cout<<"Height: "<<height<<endl;

	cout<<"Resolution: "<<resolution<<"\n";      //<<"  "<<resolution.y<<"  "<<resolution.z<<endl<<endl;

	cout<<"Ref Point: x: "<<ref_point.x<<endl;
	cout<<"Ref Point: y: "<<ref_point.y<<endl;
	cout<<"Ref Point: z: "<<ref_point.z<<endl;

}

void grid::allocate_points_to_grid (pcl::PointCloud<pcl::PointXYZRGB> cloud, pcl::PointCloud<pcl::Normal> cloud_normals)
{
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

	return;
}

void grid::allocate_points_to_grid_display( pcl::PointCloud<pcl::PointXYZRGB> cloud, pcl::PointCloud<pcl::Normal> cloud_normals, double orig_resolution)
{
	double xdiff, ydiff, zdiff;
	int xindex, yindex, zindex;

	int sub_factor = 3;

	for(int i = 0; i < cloud.points.size(); i++){

		xdiff = (cloud.points[i].x - ref_point.x)/orig_resolution ;
		ydiff = (cloud.points[i].y - ref_point.y)/orig_resolution ;
		zdiff = (cloud.points[i].z - ref_point.z)/orig_resolution ;

		xindex = (int)(xdiff);	// + 1.0f);
		yindex = (int)(ydiff);	// + 1.0f);
		zindex = (int)(zdiff);	// + 1.0f);

		if( xdiff - xindex < 0.3333)
			xindex = xindex * sub_factor;
		else if(xdiff - xindex > 0.3333 && xdiff - xindex <= 0.6667)
			xindex = xindex * sub_factor + 1;
		else
			xindex = xindex * sub_factor + 2;

		if( ydiff - yindex < 0.3333)
			yindex = yindex * sub_factor;
		else if(ydiff - yindex > 0.3333 && ydiff - yindex <= 0.6667)
			yindex = yindex * sub_factor + 1;
		else
			yindex = yindex * sub_factor + 2;


		if( zdiff - zindex < 0.3333)
			zindex = zindex * sub_factor;
		else if(zdiff - zindex > 0.3333 && zdiff - zindex <= 0.6667)
			zindex = zindex * sub_factor + 1;
		else
			zindex = zindex * sub_factor + 2;

		data[xindex][yindex][zindex].p_list->points.push_back(cloud.points[i]);
		data[xindex][yindex][zindex].n_list->points.push_back(cloud_normals.points[i]);
	}

	return;
}

void grid::remove_spurious_voxels(pcl::KdTreeFLANN<pcl::PointXYZ> &kdtree_grid)
{
	for (int i = 0; i < length; i++) {
		for (int j = 0; j < width; j++) {
			for (int k = 0; k < height; k++) {
				if (data[i][j][k].used and is_spurious(kdtree_grid, i, j, k)) {
					cout << "found spurious voxel " << i << " " << j << " " << k << endl;
					// mark this is as an unused voxel i.e. ignore it in the future
					data[i][j][k].used = false;
				}
			}
		}
	}
}

void grid::remove_voxels(int threshold, bool disp)
{
	long voxel_used_count = 0;

	for (int i = 0; i < length; i++) {
		for (int j = 0; j < width; j++) {
			for (int k = 0; k < height; k++) {
				if (data[i][j][k].p_list->points.size() >= threshold) {
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
	return;
}

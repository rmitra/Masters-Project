#ifndef HEURISTICS_HPP
#define HEURISTICS_HPP

#include <vector>
#include <pcl/kdtree/impl/kdtree_flann.hpp>

#include "block.hpp"
#include "grid.hpp"

using namespace std;

// find the number of neighbours a point has within a distance of radius
int num_neighbours(pcl::KdTreeFLANN<pcl::PointXYZ> &kdtree_grid, int i, int j, int k, int radius=1.8)
{
	pcl::PointXYZ q_point;

	q_point.x = (double)i;
	q_point.y = (double)j;
	q_point.z = (double)k;

	vector<int> pointIdxRadiusSearch;
	vector<float> pointRadiusSquaredDistance;

	int found = kdtree_grid.radiusSearch(q_point, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);
    return found;
}

// heuristic for determining if a voxel is noisy: the ratio of the number of
// neighbours it has in a distance of 2R is much less than the number of neighbours
// it has in a distance of R (here by tmuch less we mean threshold or lower)
// default value of R is 1.8
bool is_spurious(pcl::KdTreeFLANN<pcl::PointXYZ> &kdtree_grid, int i, int j, int k, int radius=1.8, int threshold=7.5)
{
    int nearer_neighbours = num_neighbours(kdtree_grid, i, j, k, radius);
    int farther_neighbours = num_neighbours(kdtree_grid, i, j, k, 2*radius);
    float ratio = (float)farther_neighbours / (float)nearer_neighbours;

    return (ratio < threshold);
}

void trim_box(Block& b, grid* g)
{
    // we will only look at those extremal elements in the 'data' field of g which
    // correspond to voxels lying within the block

    int x_lo = b.x; int y_lo = b.y; int z_lo = b.z;
    int x_hi = x_lo + b.length - 1; int y_hi = y_lo + b.width - 1; int z_hi = z_lo + b.height - 1;

    // there are eight corner grid_elements in g->data
    // these are the candidates for the grid_elements having extremal values of x,y,z

    grid_element* corners[8];

    corners[0] = &( g->data[x_lo][y_lo][z_lo] );
    corners[1] = &( g->data[x_lo][y_lo][z_hi] );
    corners[2] = &( g->data[x_lo][y_hi][z_lo] );
    corners[3] = &( g->data[x_lo][y_hi][z_hi] );
    corners[4] = &( g->data[x_hi][y_lo][z_lo] );
    corners[5] = &( g->data[x_hi][y_lo][z_hi] );
    corners[6] = &( g->data[x_hi][y_hi][z_lo] );
    corners[7] = &( g->data[x_hi][y_hi][z_hi] );

    double min_x = 1e8, min_y = 1e8, min_z = 1e8;
    double max_x = -1, max_y = -1, max_z = -1;

    // min_x = max_x = corners[1]->p_list->points[0].x;
    // min_y = max_y = corners[1]->p_list->points[0].y;
    // min_z = max_z = corners[1]->p_list->points[0].z;

    for (int i = 0 ; i < 8 ; i++) {
        auto itr = corners[i]->p_list->points.begin();
        // cout << "corner number " << i << " has " << corners[i]->p_list->points.size() << " points in it" << endl;
        for (; itr != corners[i]->p_list->points.end() ; itr++) {
            if (itr->x < min_x) min_x = itr->x;
            if (itr->y < min_y) min_y = itr->y;
            if (itr->z < min_z) min_z = itr->z;

            if (itr->x > max_x) max_x = itr->x;
            if (itr->y > max_y) max_y = itr->y;
            if (itr->z > max_z) max_z = itr->z;
        }
    }

    min_x -= g->ref_point.x; max_x -= g->ref_point.x;
    min_y -= g->ref_point.y; max_y -= g->ref_point.y;
    min_z -= g->ref_point.z; max_z -= g->ref_point.z;

    if (min_x != 0) min_x /= g->resolution; if (max_x != 0) max_x /= g->resolution;
    if (min_y != 0) min_y /= g->resolution; if (max_y != 0) max_y /= g->resolution;
    if (min_z != 0) min_z /= g->resolution; if (max_z != 0) max_z /= g->resolution;

    cout << "min_x " << min_x << " " << "min_y " << min_y << " " << "min_z " << min_z << endl;
    cout << "max_x " << max_x << " " << "max_y " << max_y << " " << "max_z " << max_z << endl;

    cout << "block min vals " << b.x << " " << b.y << " " << b.z << endl;
    cout << "block max vals " << (b.x + b.length) << " " << (b.y + b.width) << " " << (b.z + b.height) << endl;

    cout << endl;
}

// post processing sanity check step: shrink any boxes that do not tightly bound their contained points
// in other words, if a box has some slack space, remove it
void trim_boxes(vector<Block> &block_list, grid* g)
{
    // iterate over the points in a block as follows:
    // we know the extents of the block
    // using this, we iterate over the elements of the 'data' field of the grid object
    // for each grid_element in 'data':
    //     find the minimum x,y,z coordinates of the points in the grid_element
    // find the minimum over all these to get the minimum x,y,z coordinates in the block
    // trim the box dimensions to match these values

    for (auto itr = block_list.begin() ; itr != block_list.end() ; itr++) {
        // itr->display_extents();
        trim_box(*itr, g);
    }
}

// returns the center of gravity of a block given its extents
void center_of_gravity(Block& b, int &x, int &y, int &z) {
    x = b.x + (double)b.length/2;
    y = b.y + (double)b.width/2;
    z = b.z + (double)b.height/2;
}

#endif
#ifndef HEURISTICS_HPP
#define HEURISTICS_HPP

#include <vector>
#include <algorithm> // std::sort
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

struct triple {
    double x; double y; double z;
    triple() : x(0), y(0), z(0) {}
    triple(double _x, double _y, double _z) : x(_x), y(_y), z(_z) {}
};

bool compare_triples(const triple &a, const triple &b) {
    return (a.x < b.x) or
           (a.x == b.x and a.y < b.y) or
           (a.x == b.x and a.y == b.y and a.z < b.z);
}

// the base is defined as all points in a list of blocks whose
// lower y-coordinate is equal to the minimum among all lower y-coordinates
// of blocks in the block list
double find_base_y_coordinate(vector<Block> &block_list)
{
    double min_y = block_list[0].y;
    for (auto itr = block_list.begin() ; itr != block_list.end() ; itr++) {
        if (itr->y < min_y) {
            min_y = itr->y;
        }
    }
    return min_y;
}

double find_top_y_coordinate(vector<Block> &block_list)
{
    double min_y = block_list[0].y;
    for (auto itr = block_list.begin() ; itr != block_list.end() ; itr++) {
        if (itr->y > min_y) {
            min_y = itr->y;
        }
    }
    return min_y;
}

double find_base_z_coordinate(vector<Block> &block_list)
{
    double min_y = block_list[0].z;
    for (auto itr = block_list.begin() ; itr != block_list.end() ; itr++) {
        if (itr->z < min_y) {
            min_y = itr->z;
        }
    }
    return min_y;
}

double find_top_z_coordinate(vector<Block> &block_list)
{
    double min_y = block_list[0].z;
    for (auto itr = block_list.begin() ; itr != block_list.end() ; itr++) {
        if (itr->z > min_y) {
            min_y = itr->z;
        }
    }
    return min_y;
}

// find all points in the base of a list of blocks
// we could have simply looked for blocks whose lower y-coordinates were 0,
// but that would have been too specific - what if later on we wanted
// more generalised block lists?
// this will be used to find the convex hull of the base of the block list
void find_all_base_points(vector<Block> &block_list, vector<triple> &result)
{
    double min_y = find_base_y_coordinate(block_list);

    for (auto itr = block_list.begin() ; itr != block_list.end() ; itr++)
    {
        // cout << "Checking point " << itr->x << " " << itr->y << " " << itr->z << endl;
        if (itr->y == min_y)
        {
            // Add all of the base points of the block to the result
            triple base_point1(itr->x, itr->y, itr->z);
            triple base_point2(itr->x + itr->length, itr->y, itr->z);
            triple base_point3(itr->x + itr->length, itr->y, itr->z + itr->height);
            triple base_point4(itr->x, itr->y, itr->z + itr->height);
            result.push_back(base_point1);
            result.push_back(base_point2);
            result.push_back(base_point3);
            result.push_back(base_point4);
        }
    }
}

// cross product of two 2d points
// in which the y-coordinate is the same
double cross(triple o, triple a, triple b) {
    return (a.x - o.x)*(b.z - o.z) - (a.z - o.z)*(b.x - o.x);
}

// find the convex hull of a vector of points
// for this, we use andrew's monotone chain algorithm
// note: the algorithm is for 2d points, and these points are 3d
// however, this is not an issue because the points we will be
// applying this algo to all have the same y-coordinate (base points)
// so effectively they are 2d
void find_convex_hull(vector<triple> &points, vector<triple> &result)
{
    if (points.size() <= 1)
        return;

    sort(points.begin(), points.end(), compare_triples);

    // Build lower hull
    for (auto p : points) {
        while (result.size() >= 2 and cross(result[result.size()-2], result[result.size()-1], p) <= 0) {
            result.pop_back();
        }
        result.push_back(p);
    }
 
    // Build upper hull
    vector<triple> upper;
    for (auto itr = points.rbegin(); itr != points.rend() ; itr++) {
        while (upper.size() >= 2 and cross(upper[upper.size()-2], upper[upper.size()-1], *itr) <= 0) {
            upper.pop_back();
        }
        upper.push_back(*itr);
    }
 
    // Concatenation of the lower and upper hulls gives the convex hull.
    // Last point of each list is omitted because it is repeated at the beginning of the other list.
    result.pop_back();
    upper.pop_back();
    result.insert(result.end(), upper.begin(), upper.end());
}

void find_convex_hull_of_base(vector<Block> &block_list, vector<triple> &result)
{
    vector<triple> base_points;
    find_all_base_points(block_list, base_points);
    // cout << "Base points" << endl;
    // for (triple t : base_points) {
    //     cout << t.x << " " << t.y << " " << t.z << endl;
    // }
    find_convex_hull(base_points, result);
}

// fills in the center of gravity of the entire block list into x, y and z
// assumes that each block in the block list has the same density
// cog: center of gravity
void find_center_of_gravity(vector<Block> &block_list, triple &cog)
{
    double total_volume = 0;
    for (auto itr = block_list.begin() ; itr != block_list.end() ; itr++)
    {
        double block_volume = itr->volume();
        total_volume += block_volume;

        double x, y, z;
        itr->find_center_of_gravity(x, y, z);
        // cout << "COG of block: " << x << " " << y << " " << z << endl;

        cog.x += block_volume * x;
        cog.y += block_volume * y;
        cog.z += block_volume * z;
    }

    // cout << "COG before division: " << cog.x << " " << cog.y << " " << cog.z << endl;

    // cout << "Total volume " << total_volume << endl;
    cog.x /= total_volume;
    cog.y /= total_volume;
    cog.z /= total_volume;

    // cout << "COG after division: " << cog.x << " " << cog.y << " " << cog.z << endl;
}

// code to check if a point is in a polygon, not necessarily convex
// (although in this specific application, the polygon is surely convex)
// assumption: the point as well as all vertices in the polygon
// have the same y-coordinate
// p: point, g: polygon
bool is_point_inside_polygon(triple &p, vector<triple> &g)
{
    // code taken from: http://stackoverflow.com/q/11716268
    int i, j, nvert = g.size();
    bool is_inside = false;
    for (i = 0, j = nvert-1; i < nvert; j = i++) {
        if (((g[i].z > p.z) != (g[j].z > p.z)) and
            (p.x - g[i].x < (g[j].x - g[i].x) * (p.z - g[i].z) / (g[j].z - g[i].z))) {
            cout << "flip " << g[i].x << " " << g[i].z << " " << g[j].x << " " << g[j].z << endl;
            is_inside = not(is_inside);
        }
    }
    return is_inside;
    // high-level explanation: we send a ray parallel to the z-axis and look at each edge and, if the point is within the scope
    // of the y-coordinates of the edge, 
    // explanation: the if condition has two parts: either the ray does not intersect the edge, or it does and the point is
    // to the left of the edge
}

// Check if the point (x, b.y + b.height, z) lies on the top face of block b.
// This is a special case of is_point_inside_polygon.
static inline bool check_point_in_top_face(double x, double z, Block& b)
{
    return (b.x <= x and x <= b.x + b.length) and (b.z <= z and z <= b.z + b.height);
}

// This function is called only when we know that b1 is on top of b2.
// This function checks if the bottom face of b1 overlaps with
// the top face of b2.
bool check_if_faces_overlap(Block &b1, Block &b2)
{
    bool x1z1 = check_point_in_top_face(b1.x, b1.z, b2);
    bool x1z2 = check_point_in_top_face(b1.x + b1.length, b1.z, b2);
    bool x2z1 = check_point_in_top_face(b1.x, b1.z + b1.height, b2);
    bool x2z2 = check_point_in_top_face(b1.x + b1.length, b1.z + b1.height, b2);
    return x1z1 or x1z2 or x2z1 or x2z2;
}

// return true if one of the base points of b1 is directly
// on top of b2's top, or vice versa
bool is_on_top_of(Block& b1, Block& b2)
{
    bool b1_on_top = b1.y == b2.y + b2.width;
    bool b2_on_top = b2.y == b1.y + b1.width;
    if (not(b1_on_top or b2_on_top)) {
        return false;
    }
    else {
        return b1_on_top ? check_if_faces_overlap(b1, b2) : check_if_faces_overlap(b2, b1);
    }
}

// update touches_ground for blocks in a block list
// algorithm: sort the blocks by y-coordinate and start from the lowest block
void update_touches_ground(vector<Block> &block_list)
{
    // base of the entire building
    double base = find_base_y_coordinate(block_list);

    // build a map of top y coordinate to blocks, sorted by the keys
    // note: we map to the index in the block_list instead of the block itself
    // as this is more efficient
    map<double, vector<int> > y_to_blocks;
    for (int i = 0 ; i < block_list.size() ; i++) {
        double y = block_list[i].y + block_list[i].width;
        y_to_blocks[y].push_back(i);
    }
    // perform the update
    for (auto &itr : y_to_blocks) {
        // cout << "height: " << itr.first << endl;
        for (auto &itr2 : itr.second) {
            if (block_list[itr2].y == base) {
                block_list[itr2].touches_ground = true;
            }
            else {
                // check if it rests on a block which recursively touches the ground
                // do this for each one of the blocks on which it rests
                double curr_base = block_list[itr2].y;
                auto itr3 = y_to_blocks.find(curr_base);
                if (itr3 != y_to_blocks.end()) {
                    // it does rest on some blocks, so check each one of them
                    for (auto const &itr4 : itr3->second) {
                        if (block_list[itr4].touches_ground and check_if_faces_overlap(block_list[itr2], block_list[itr4])) {
                            block_list[itr2].touches_ground = true;
                            break;
                        }
                    }
                }
            }
        }
    }
}

// For a building to be stable, check_center_of_gravity_inside() should return true
// And also, each block must either touch the ground or lie on a block which
// recursively touches the ground
bool check_stability(vector<Block> &block_list)
{
    update_touches_ground(block_list);
    // check for blocks which still have touches_ground == false
    for (auto const &block : block_list) {
        // cout << "block at " << block.y << " " << (block.touches_ground ? "touches" : "doesn't touch") << " ground\n";
        if (not block.touches_ground) {
            return false;
        }
    }
    return true;
}

// Check if the center of gravity is inside the convex hull the bases
// of all the blocks which directly touch the ground
bool check_center_of_gravity_inside(vector<Block> &block_list)
{
    triple cog;
    find_center_of_gravity(block_list, cog);

    // for (auto block : block_list) {
    //     cout << block.x << " " << block.y << " " << block.z << " " << endl;
    // }

    // cout << "Center of gravity " << cog.x << " " << cog.y << " " << cog.z << endl;

    // project the center of gravity down to the base
    cog.y = find_base_y_coordinate(block_list);

    // cout << "Center of gravity projected " << cog.x << " " << cog.y << " " << cog.z << endl;

    vector<triple> convex_hull_of_base;
    find_convex_hull_of_base(block_list, convex_hull_of_base);

    // cout << "Convex hull of base" << endl;
    // for (triple t : convex_hull_of_base) {
    //     cout << t.x << " " << t.y << " " << t.z << endl;
    // }

    bool is_inside = is_point_inside_polygon(cog, convex_hull_of_base);
    cout << "Center of gravity is " << (is_inside ? "inside" : "not inside") << " base" << endl;

    return is_inside;
}

#endif
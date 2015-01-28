#include <iostream>
#include <cstdio>
#include <cmath>
#include <vector>
#include <boost/algorithm/string.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/point_types.h>
#include <pcl/features/shot_omp.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/correspondence.h>
#include <pcl/features/board.h>
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/common/transforms.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/our_cvfh.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/vfh.h>


#include "point_res.cpp"

using namespace std;
using namespace boost;

#ifndef EXT_FEAT
#define EXT_FEAT

double get_cloud_resolution(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
	double cloud_res = computeCloudResolution(cloud);
	cerr << "Point res: " << cloud_res<<"\n";

	return cloud_res;
}

void print_aspect_ratio(string model_name, Block &b, int model_number, bool is_splited, int plane, int cut_index, Block &part1, Block &part2)
{
	int l, l_1, l_2;

	if(b.length > b.width){

		if(b.length > b.height){
			l = b.length;

			if(b.height > b.width){
				l_1 = b.height;
				l_2 = b.width;
			}
			else{
				l_2 = b.height;
				l_1 = b.width;
			}
		}
		else{
			l = b.height;
			l_1 = b.length;
			l_2 = b.width;
		}
	}
	else{

		if(b.width > b.height){
			l = b.width;

			if(b.height > b.width){
				l_1 = b.height;
				l_2 = b.length;
			}
			else{
				l_2 = b.height;
				l_1 = b.length;
			}
		}
		else{
			l = b.height;
			l_1 = b.width;
			l_2 = b.length;
		}

	}

	//model_name += "_";
	//model_name += to_string(model_number);

	cout << model_name << " " << model_number << " ";

	cout << (double)l_1/(double)l << " " << (double)l_2/(double)l << " " << (double)l_2/(double)l_1 << " ";

	if(!is_splited)
		cout << "-1" << "\n";
	else{

		cout << "1 ";

		double x = (double)( part1.x - b.x ) / (double)b.length;
		double y = (double)( part1.y - b.y ) / (double)b.width;
		double z = (double)( part1.z - b.z ) / (double)b.height;

		double l = (double)( part1.length ) / (double)b.length;
		double w = (double)( part1.width ) / (double)b.width;
		double h = (double)( part1.height ) / (double)b.height;

		cout << x << " " << y << " " << z << " " << l << " " << w << " " << h << " ";

		x = (double)( part2.x - b.x ) / (double)b.length;
		y = (double)( part2.y - b.y ) / (double)b.width;
		z = (double)( part2.z - b.z ) / (double)b.height;

		l = (double)( part2.length ) / (double)b.length;
		w = (double)( part2.width ) / (double)b.width;
		h = (double)( part2.height ) / (double)b.height;

		cout << x << " " << y << " " << z << " " << l << " " << w << " " << h;

		cout << "\n";
	}
}

pcl::PointCloud<pcl::PointXYZ>::Ptr get_keypoints(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, pcl::PointCloud<pcl::Normal>::Ptr &cloud_normals, double cloud_res, double mult)
{
	/*
	pcl::PointCloud<pcl::PointNormal>::Ptr keypoints (new pcl::PointCloud<pcl::PointNormal>());
	pcl::ISSKeypoint3D<pcl::PointNormal, pcl::PointNormal> iss_detector;

	pcl::search::KdTree<pcl::PointNormal>::Ptr tree (new pcl::search::KdTree<pcl::PointNormal> ());

	double iss_salient_radius_ = 6 * cloud_res;
	double iss_non_max_radius_ = 4 * cloud_res;

	iss_detector.setSearchMethod(tree);

	iss_detector.setSalientRadius(iss_salient_radius_);
	iss_detector.setNonMaxRadius(iss_non_max_radius_);

	iss_detector.setMinNeighbors(5);
	iss_detector.setNumberOfThreads(4);

	iss_detector.setInputCloud (cloud);
	iss_detector.compute (*keypoints);
	*/

	double cloud_sr = mult * cloud_res;

	pcl::PointCloud<int> sampled_indices;
	pcl::PointCloud<pcl::PointXYZ>:: Ptr cloud_keypoints(new pcl::PointCloud<pcl::PointXYZ>());

	pcl::UniformSampling<pcl::PointXYZ> uniform_sampling;
	uniform_sampling.setInputCloud (cloud);
	uniform_sampling.setRadiusSearch (cloud_sr);
	uniform_sampling.compute (sampled_indices);
	pcl::copyPointCloud (*cloud, sampled_indices.points, *cloud_keypoints);
	cerr << "Cloud total points: " << cloud->size () << "; Selected Keypoints: " << cloud_keypoints->size () << std::endl;

	return cloud_keypoints;
}

pcl::PointCloud<pcl::SHOT352>::Ptr get_features(pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr cloud_normals, double cloud_res){

	pcl::SHOTEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::SHOT352> describer;
	//pcl::VFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::VFHSignature308> vfh;
	pcl::PointCloud<pcl::SHOT352>::Ptr descriptors (new pcl::PointCloud<pcl::SHOT352>);

	/*
	vfh.setInputCloud (cloud);
    vfh.setInputNormals (cloud_normals);

	pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhs (new pcl::PointCloud<pcl::VFHSignature308> ());

	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>() );

	vfh.setSearchMethod (tree);


	vfh.compute(*vfhs);

	cerr<<"Descriptor cloud size :"<<(vfhs->points).size()<<"\n";

	return vfhs;
	*/

	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

	describer.setSearchMethod (tree);
	describer.setKSearch(0);
	describer.setRadiusSearch(15*cloud_res);

	describer.setInputCloud (keypoints);
	describer.setInputNormals (cloud_normals);
	describer.setSearchSurface (cloud);
	//describer.setNormalizeBins(false);

	describer.compute(*descriptors);

	cerr<<"Descriptor cloud size :"<<descriptors->size()<<"\n";

	return descriptors;

}

pcl::CorrespondencesPtr get_no_corresspondence(pcl::PointCloud<pcl::SHOT352>::Ptr &model_des, pcl::PointCloud<pcl::SHOT352>::Ptr &test_des){

	pcl::KdTreeFLANN<pcl::SHOT352> match_corr;
	match_corr.setInputCloud (model_des);

	pcl::CorrespondencesPtr model_scene_corrs (new pcl::Correspondences ());

	for (size_t i = 0; i < test_des->size (); ++i)
	{
		std::vector<int> neigh_indices (1);
		std::vector<float> neigh_sqr_dists (1);

		//if (!pcl_isfinite (model_des->at(i).descriptor[0])) //skipping NaNs
		//{
			//continue;
		//}

		int found_neighs = match_corr.nearestKSearch (test_des->at(i), 1, neigh_indices, neigh_sqr_dists);

		if(found_neighs == 1 && neigh_sqr_dists[0] < 0.25f) //  add match only if the squared descriptor distance is less than 0.25 (SHOT descriptor distances are between 0 and 1 by design)
		{
			pcl::Correspondence corr (neigh_indices[0], static_cast<int> (i), neigh_sqr_dists[0]);
			model_scene_corrs->push_back (corr);

		}
	}

	return model_scene_corrs;
}

std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > get_clustering(pcl::CorrespondencesPtr model_scene_corrs, pcl::PointCloud<pcl::PointXYZ>::Ptr model_keypoints, pcl::PointCloud<pcl::PointXYZ>::Ptr scene_keypoints, pcl::PointCloud<pcl::PointXYZ>::Ptr model, pcl::PointCloud<pcl::PointXYZ>::Ptr scene, pcl::PointCloud<pcl::Normal>::Ptr model_normals, pcl::PointCloud<pcl::Normal>::Ptr scene_normals, double model_res){

	std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations;
	std::vector<pcl::Correspondences> clustered_corrs;

	double rf_rad = 10 * model_res;
	int cg_size = 10 * model_res;
	double cg_thresh = 5.0;


	pcl::PointCloud<pcl::ReferenceFrame>::Ptr model_rf (new pcl::PointCloud<pcl::ReferenceFrame> ());
    pcl::PointCloud<pcl::ReferenceFrame>::Ptr test_scene_rf (new pcl::PointCloud<pcl::ReferenceFrame> ());

    pcl::BOARDLocalReferenceFrameEstimation<pcl::PointXYZ, pcl::Normal, pcl::ReferenceFrame> rf_est;
    rf_est.setFindHoles (true);
    rf_est.setRadiusSearch (rf_rad);

    rf_est.setInputCloud (model_keypoints);
    rf_est.setInputNormals (model_normals);
    rf_est.setSearchSurface (model);
    rf_est.compute (*model_rf);

    rf_est.setInputCloud (scene_keypoints);
    rf_est.setInputNormals (scene_normals);
    rf_est.setSearchSurface (scene);
    rf_est.compute (*test_scene_rf);

    //  Clustering

    //pcl::PointCloud<pcl::PointXYZ>::Ptr model_key_xyz (new pcl::PointCloud<pcl::PointXYZ>());
	//pcl::copyPointCloud(*model_keypoints, *model_key_xyz);

    //pcl::PointCloud<pcl::PointXYZ>::Ptr scene_key_xyz (new pcl::PointCloud<pcl::PointXYZ>());
	//pcl::copyPointCloud(*scene_keypoints, *scene_key_xyz);

    pcl::Hough3DGrouping<pcl::PointXYZ, pcl::PointXYZ, pcl::ReferenceFrame, pcl::ReferenceFrame> clusterer;

    clusterer.setHoughBinSize (cg_size);
    clusterer.setHoughThreshold (cg_thresh);
    clusterer.setUseInterpolation (true);
    clusterer.setUseDistanceWeight (false);

    clusterer.setInputCloud (model_keypoints);
    clusterer.setInputRf (model_rf);
    clusterer.setSceneCloud (scene_keypoints);
    clusterer.setSceneRf (test_scene_rf);
    clusterer.setModelSceneCorrespondences (model_scene_corrs);

    //clusterer.cluster (clustered_corrs);
    clusterer.recognize (rototranslations, clustered_corrs);
	cerr<<"Rot size "<<rototranslations.size()<<" cluster_corrs size "<<clustered_corrs.size()<<"\n";

	for(int i = 0; i < rototranslations.size(); i++)
		cerr<<"Cluster corr "<<i<<" size :"<<clustered_corrs[i].size()<<"\n";

	return rototranslations;

}

/*int main(int argc, char **argv){

	if(argc < 3){
		cout<<"Usage: <executabel binary> <path to .ply file> <Scale for rendering>\n";
		exit(0);
	}

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);	 // new point cloud variable
	pcl::PointCloud<pcl::PointXYZ>::Ptr test_cloud_1 = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals;
	pcl::PointCloud<pcl::Normal>::Ptr test_cloud_1_normals;

	if (pcl::io::loadPLYFile<pcl::PointXYZ> (argv[1], *cloud) == -1)      			// loading the .ply file into cloud
	{
		PCL_ERROR ("Couldn't read .ply file\n");
		exit(0);
	}

	if (pcl::io::loadPLYFile<pcl::PointXYZ> (argv[2], *test_cloud_1) == -1)      			// loading the .ply file into cloud
	{
		PCL_ERROR ("Couldn't read .ply file\n");
		exit(0);
	}

	double cloud_res = get_cloud_resolution(cloud);
	double test_cloud_1_res = get_cloud_resolution(test_cloud_1);

	compute_normal(cloud, cloud_normals);
	compute_normal(test_cloud_1, test_cloud_1_normals);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_keys = get_keypoints(cloud, cloud_normals, cloud_res, 8.0);
	pcl::PointCloud<pcl::PointXYZ>::Ptr test_cloud_1_keys = get_keypoints(test_cloud_1, test_cloud_1_normals, test_cloud_1_res, 8.0);
	//pcl::PointCloud<pcl::PointNormal>::Ptr test_cloud_2_keys = get_keypoints(test_cloud_2, test_cloud_2_res);

	pcl::PointCloud<pcl::SHOT352>::Ptr cloud_descriptor = get_features(cloud_keys, cloud, cloud_normals, cloud_res);
	pcl::PointCloud<pcl::SHOT352>::Ptr test_descriptor_1 = get_features(test_cloud_1_keys, test_cloud_1, test_cloud_1_normals, test_cloud_1_res);
	//pcl::PointCloud<pcl::SHOT352>::Ptr test_descriptor_2 = get_features(test_cloud_2_keys, test_cloud_2, test_cloud_2_res);

	pcl::CorrespondencesPtr match_corr_1 = get_no_corresspondence(cloud_descriptor, test_descriptor_1);
	//pcl::CorrespondencesPtr match_corr_2 = get_no_corresspondence(cloud_descriptor, test_descriptor_2);

	cerr<<"For Match 1 "<<match_corr_1->size()<<"\n";	//<<" For Match 2	"<<match_corr_2->size()<<"\n";

	std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rot_trans_1 = get_clustering(match_corr_1, cloud_keys, test_cloud_1_keys, cloud, test_cloud_1, cloud_normals, test_cloud_1_normals, cloud_res);
	//std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rot_trans_2 = get_clustering(match_corr_2, cloud_keys, test_cloud_2_keys, cloud, test_cloud_2, cloud_res);

	cerr<<"MAT SIZE FOR 1 :"<<rot_trans_1.size()<<"\n";
	//cerr<<"MAT SIZE FOR 2 :"<<rot_trans_2.size()<<"\n";

	return 0;

}*/

string extract_name(char * filename){

	string temp(filename);

	vector<string> fields_temp;
	vector<string> fields;

	split(fields_temp, temp, is_any_of("/") );

	split(fields, fields_temp[fields_temp.size() - 1], is_any_of(".") );

	cerr<<"Extracted model name :"<<fields[0]<<"\n";

	return fields[0];

}

void get_cloud_from_grid(Block b, grid *g, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, pcl::PointCloud<pcl::Normal>::Ptr &cloud_normals){

	cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
	cloud_normals = pcl::PointCloud<pcl::Normal>::Ptr(new pcl::PointCloud<pcl::Normal>);

	for(int i = b.x; i < b.x + b.length; i++){
		for(int j = b.y; j < b.y + b.width; j++){
			for(int k = b.z; k < b.z + b.height; k++){
				if(g->data[i][j][k].used){

					pcl::PointCloud<pcl::PointXYZ>::Ptr cl_xyz (new pcl::PointCloud<pcl::PointXYZ>());
					pcl::copyPointCloud(*( (g->data[i][j][k]).p_list ), *cl_xyz);

					*cloud += *(cl_xyz);
					*cloud_normals += *((g->data[i][j][k]).n_list);
				}
			}
		}
	}

}

void save_features(string dir_name, string model_name, int sub_model_count, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, pcl::PointCloud<pcl::Normal>::Ptr &cloud_normals){

	double cloud_res = get_cloud_resolution(cloud);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_keys = get_keypoints(cloud, cloud_normals, cloud_res, 5.0);

	pcl::PointCloud<pcl::SHOT352>::Ptr cloud_descriptor = get_features(cloud_keys, cloud, cloud_normals, cloud_res);

	model_name += "_";
	model_name += to_string(sub_model_count);

	string model_name_des = dir_name + model_name + "_des.pcd";

	pcl::io::savePCDFileASCII (model_name_des, *cloud_descriptor);

	string model_name_key = dir_name + model_name + "_key.pcd";

	pcl::io::savePCDFileASCII (model_name_key, *cloud_keys);

	string model_name_cloud = dir_name + model_name + "_cloud.pcd";

	pcl::io::savePCDFileASCII (model_name_cloud, *cloud);

	string model_name_normal = dir_name + model_name + "_normal.pcd";

	pcl::io::savePCDFileASCII (model_name_normal, *cloud_normals);


}



#endif

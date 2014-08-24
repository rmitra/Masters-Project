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
#include <flann/flann.h>
//#include <flann/io/hdf5.h>


#include "point_res.cpp"

using namespace std;
using namespace boost;

#ifndef EXT_FEAT
#define EXT_FEAT

double get_cloud_resolution(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud){

	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz (new pcl::PointCloud<pcl::PointXYZ>());
	//pcl::copyPointCloud(*cloud, *cloud_xyz);
	
	double cloud_res = computeCloudResolution(cloud); 
	cerr<<"Point res: "<<cloud_res<<"\n";

	return cloud_res;
}


pcl::PointCloud<pcl::PointXYZ>::Ptr get_keypoints(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, pcl::PointCloud<pcl::Normal>::Ptr &cloud_normals, double cloud_res, double mult){

	
	double cloud_sr = mult * cloud_res;
	
	pcl::PointCloud<int> sampled_indices;
	pcl::PointCloud<pcl::PointXYZ>:: Ptr cloud_keypoints(new pcl::PointCloud<pcl::PointXYZ>());

	pcl::UniformSampling<pcl::PointXYZ> uniform_sampling;
	uniform_sampling.setInputCloud (cloud);
	uniform_sampling.setRadiusSearch (cloud_sr);
	uniform_sampling.compute (sampled_indices);
	pcl::copyPointCloud (*cloud, sampled_indices.points, *cloud_keypoints);
	//cerr<< "Cloud total points: " << cloud->size () << "; Selected Keypoints: " << cloud_keypoints->size () << std::endl;
	
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



std::vector<pcl::Correspondences> get_clustering(pcl::CorrespondencesPtr model_scene_corrs, pcl::PointCloud<pcl::PointXYZ>::Ptr model_keypoints, pcl::PointCloud<pcl::PointXYZ>::Ptr scene_keypoints, pcl::PointCloud<pcl::PointXYZ>::Ptr model, pcl::PointCloud<pcl::PointXYZ>::Ptr scene, pcl::PointCloud<pcl::Normal>::Ptr model_normals, pcl::PointCloud<pcl::Normal>::Ptr scene_normals, double model_res){	

	std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations;
	std::vector<pcl::Correspondences> clustered_corrs;
	
	double rf_rad = 30 * model_res;
	int cg_size = 15 * model_res;
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
	//cerr<<"Rot size "<<rototranslations.size()<<" cluster_corrs size "<<clustered_corrs.size()<<"\n";
	
	//for(int i = 0; i < rototranslations.size(); i++)
	//	cerr<<"Cluster corr "<<i<<" size :"<<clustered_corrs[i].size()<<"\n";
	
	return clustered_corrs;

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
					*cloud += *((g->data[i][j][k]).p_list);
					*cloud_normals += *((g->data[i][j][k]).n_list);
				}		
			}
		}
	}
	
}

/*
void save_features(string dir_name, string model_name, int sub_model_count, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, pcl::PointCloud<pcl::Normal>::Ptr &cloud_normals){

	double cloud_res = get_cloud_resolution(cloud);
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_keys = get_keypoints(cloud, cloud_normals, cloud_res, 8.0);

	pcl::PointCloud<pcl::SHOT352>::Ptr cloud_descriptor = get_features(cloud_keys, cloud, cloud_normals, cloud_res);
	
	model_name += "_";
	model_name += to_string(sub_model_count);
	
	string model_name_ft = dir_name + model_name + ".pcd";
		
	pcl::io::savePCDFileASCII (model_name_ft, *cloud_descriptor);
	

}
*/


pcl::KdTreeFLANN<pcl::PointXYZ> load_kd_tree(vector<Training_block> data_list){
	
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	
	cloud->width = data_list.size();
	cloud->height = 1;
	cloud->points.resize(cloud->width * cloud->height);
	
	for(int i = 0; i < cloud->points.size(); i++){
		
		cloud->points[i].x = data_list[i].asp_1;
		cloud->points[i].y = data_list[i].asp_2;
		cloud->points[i].z = data_list[i].asp_3;
	}
	
	kdtree.setInputCloud (cloud);
	
	return kdtree;

}


vector<int> search_data(pcl::KdTreeFLANN<pcl::PointXYZ> &kdtree, double radius, pcl::PointXYZ search_point, vector<Training_block> &data_list, Training_block &parent){
	
	vector<int> pointIdxRadiusSearch;
	vector<float> pointRadiusSquaredDistance;

	vector<int> filtered;
	
	int found_instances = kdtree.radiusSearch (search_point, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);
	
	cerr<<"No instances found :"<<found_instances<<"\n";
	
	for (size_t i = 0; i < pointIdxRadiusSearch.size (); ++i){
      cerr << "Model Name :   "<<data_list[ pointIdxRadiusSearch[i] ].model_name <<"  " <<   data_list[ pointIdxRadiusSearch[i] ].asp_1 
                << " " <<  data_list[ pointIdxRadiusSearch[i] ].asp_2 
                << " " <<  data_list[ pointIdxRadiusSearch[i] ].asp_3 
                << " (squared distance: " << pointRadiusSquaredDistance[i] << ")" << std::endl;
	
		//if(parent.model_number == -1 || ( data_list[ pointIdxRadiusSearch[i] ].model_name.compare(parent.model_name) == 0 && data_list[ pointIdxRadiusSearch[i] ].model_number > parent.model_number ) ){
			filtered.push_back(pointIdxRadiusSearch[i]);
		//	cerr<<"Filtered model :"<<data_list[ pointIdxRadiusSearch[i] ].model_name<<" "<<data_list[ pointIdxRadiusSearch[i] ].model_number<<"\n";
		//}
	}
	
	
	return  filtered;

}


int get_match_model(vector<Training_block> &tr_data_list, vector<int> &indices, pcl::PointCloud<pcl::PointXYZ>:: Ptr &cloud, pcl::PointCloud<pcl::Normal>:: Ptr &cloud_normals, double cloud_res, char * ft_dir_name){

	vector<int> selected_indices;
	vector< std::vector<pcl::Correspondences> > clustered_corrs_list;
	int key_size;
	
	string dir_name(ft_dir_name);
	
	pcl::PointCloud<pcl::SHOT352>::Ptr repo_model_des;
	pcl::PointCloud<pcl::PointXYZ>::Ptr repo_model_cloud;
	pcl::PointCloud<pcl::Normal>::Ptr repo_model_normals;
	pcl::PointCloud<pcl::PointXYZ>::Ptr repo_model_key;
		
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_keys = get_keypoints(cloud, cloud_normals, cloud_res, 5.0);
		
	pcl::PointCloud<pcl::SHOT352>::Ptr cloud_descriptor = get_features(cloud_keys, cloud, cloud_normals, cloud_res);
	
	key_size = cloud_descriptor->size();
	
	cerr<<"Descriptor Size of Scene : "<<key_size<<"\n";
	
	
	for(int i = 0; i < indices.size(); i++){
				
		repo_model_des = pcl::PointCloud<pcl::SHOT352>::Ptr(new pcl::PointCloud<pcl::SHOT352>);
		repo_model_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
		repo_model_normals = pcl::PointCloud<pcl::Normal>::Ptr(new pcl::PointCloud<pcl::Normal>);
		repo_model_key = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
		
		
		string file_prefix = dir_name + string( tr_data_list[ indices[i] ].model_name ) + "_" + to_string( tr_data_list[ indices[i] ].model_number ) ;
		string file_name = file_prefix + "_des.pcd";
	
		if (pcl::io::loadPCDFile<pcl::SHOT352> (file_name, *repo_model_des) == -1)      			// loading the .ply file into cloud
		{
			PCL_ERROR ("Couldn't read .ply file\n");
			exit(0);
		}
		
		file_name = file_prefix + "_cloud.pcd";
		
		if (pcl::io::loadPCDFile<pcl::PointXYZ> (file_name, *repo_model_cloud) == -1)      			// loading the .ply file into cloud
		{
			PCL_ERROR ("Couldn't read .ply file\n");
			exit(0);
		}
		
		file_name = file_prefix + "_normal.pcd";
		
		if (pcl::io::loadPCDFile<pcl::Normal> (file_name, *repo_model_normals) == -1)      			// loading the .ply file into cloud
		{
			PCL_ERROR ("Couldn't read .ply file\n");
			exit(0);
		}
		
		file_name = file_prefix + "_key.pcd";
		
		if (pcl::io::loadPCDFile<pcl::PointXYZ> (file_name, *repo_model_key) == -1)      			// loading the .ply file into cloud
		{
			PCL_ERROR ("Couldn't read .ply file\n");
			exit(0);
		}
				
		//double cloud_res = get_cloud_resolution(cloud);
		
		pcl::CorrespondencesPtr match_corr = get_no_corresspondence(repo_model_des, cloud_descriptor);
		
		cerr<<"Match Corr Size: "<<match_corr->size()<<"\n";
		
		std::vector<pcl::Correspondences> clustered_corrs = get_clustering(match_corr, repo_model_key, cloud_keys, repo_model_cloud, cloud, repo_model_normals, cloud_normals, cloud_res);
		cerr<<"No. Cluster :"<<clustered_corrs.size()<<"\n";
		
		if(clustered_corrs.size() > 0 && clustered_corrs.size() <= 5){
			selected_indices.push_back(indices[i]);
			clustered_corrs_list.push_back(clustered_corrs);	
	
		}
	
		cerr<<"Done for checking "<<i<<"\n";
	}

		
	int min_size_of_list = 999;
	int best_index = -1;
	int total_size = -1;
	
	bool found_model;
		
	for(int i = 0; i < clustered_corrs_list.size(); i++){
		
		//found_model = false;
				
		/*
		for(int j = 0; j < clustered_corrs_list[i].size(); j++){
			
			cerr<<"Cluster size: "<<(clustered_corrs_list[i]).at(j).size()<<"\n";
			
			
			if( (clustered_corrs_list[i]).at(j).size() > 0.25 * (double)key_size){
				if(!found_model)
					found_model = true;
				else{
					found_model = false;
					break;
				}	
					
			}
			
		}*/	
		
		if(clustered_corrs_list[i].size() < min_size_of_list){
			min_size_of_list = clustered_corrs_list[i].size();
			best_index = selected_indices[i];	
			total_size = 0;
			
			for(int j = 0; j < clustered_corrs_list[i].size(); j++)
			{
				total_size = total_size + (clustered_corrs_list[i]).at(j).size();
			}
		
		}
		else if(clustered_corrs_list[i].size() == min_size_of_list ){
							
			int temp_total_size = 0;
				
			for(int j = 0; j < clustered_corrs_list[i].size(); j++)
			{
				temp_total_size = temp_total_size + (clustered_corrs_list[i]).at(j).size();
			}
		
			if(temp_total_size > total_size){
				best_index = i;
				total_size = temp_total_size;
			}
		
		}
	}
	return best_index;
	//return -1;
}

/*
void load_descriptors(char * dir_name, vector<Training_block> &model_data_list, vector< vector<float> > &model_hists){

	string model_name;
	
	for(int i = 0; i < model_data_list.size(); i++){
		model_name = string(dir_name) + model_data_list[i].model_name + "_des.pcd";
				
		pcl::PCLPointCloud2 model_cloud;
		int version;
		Eigen::Vector4f origin;
		Eigen::Quaternionf orientation;
		pcl::PCDReader r;
		int type; unsigned int idx;
		r.readHeader (model_name, model_cloud, origin, orientation, version, type, idx);
		int vfh_idx = pcl::getFieldIndex (model_cloud, "vfh");
		
		pcl::PointCloud <pcl::VFHSignature308> point;
		
		pcl::io::loadPCDFile (model_name, point);
		
		
		std::vector <pcl::PCLPointField> fields;
		pcl::getFieldIndex (point, "vfh", fields);
		
		
		//cerr<<"Model name :"<<model_name<<" vfh_idx value: "<<vfh_idx<<"with count : "<<fields[vfh_idx].count<<"\n";		
		
		vector<float> hist;
		hist.resize(308);
		
		for (size_t i = 0; i < fields[vfh_idx].count; ++i)
		{
			hist[i] = point.points[0].histogram[i];
		}	
	
		model_hists.push_back(hist);
	
	}

}

//void load_kdtree_index(flann::Index<flann::ChiSquareDistance<float> > &index, flann::Matrix<float> &model_data_flann){
	
	//index = flann::Index<flann::ChiSquareDistance<float> >(model_data_flann, flann::KDTreeIndexParams (4));
	//index.buildIndex();

//}


int get_match_model(flann::Index<flann::ChiSquareDistance<float> > &index, pcl::PointCloud<pcl::PointXYZ>::Ptr &scene_cloud, pcl::PointCloud<pcl::Normal>::Ptr &scene_cloud_normal, double scene_cloud_res, vector<Training_block> &model_data_list ){

	pcl::PointCloud<pcl::VFHSignature308>::Ptr p = get_features(scene_cloud, scene_cloud_normal, scene_cloud_res); 

	vector<float> hist;
	hist.resize(308);
		
	for (size_t i = 0; i < 308; ++i)
	{
		hist[i] = (p->points[0]).histogram[i];
	}
	
	flann::Matrix<float> q = flann::Matrix<float>(new float[hist.size ()], 1, hist.size ());
	memcpy (&q.ptr ()[0], &hist[0], q.cols * q.rows * sizeof (float));

	flann::Matrix<int> indices;
	flann::Matrix<float> distances;
	
	int k = 5;
	
	indices = flann::Matrix<int>(new int[k], 1, k);
    distances = flann::Matrix<float>(new float[k], 1, k);

	index.knnSearch (q, indices, distances, k, flann::SearchParams (512));
	
	for(int i = 0; i < k; i++){
		cerr<<"Model Name :"<<model_data_list[ indices[0][i] ].model_name<<" dist :"<<distances[0][i]<<"\n";
	}

	return -1;

}
*/

#endif	

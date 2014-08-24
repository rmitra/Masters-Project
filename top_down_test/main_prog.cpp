#include "split_driver.cpp"
//#include "merge_block.cpp"
//#include "prune_tree.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;	// Also required by draw_object.cpp

#include "./draw_object.cpp"


using namespace std;

int main(int argc, char **argv){

	if(argc < 3){
		cout<<"Usage: <executabel binary> <path to .ply file> <feature_directory> <Scale for rendering>\n";
		exit(0);
	}
	
	cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);	 // new point cloud variable
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals;
	
	pcl::PointXYZ min, max;		// To store min max
	
	int length, width, height;
	
	double resolution;
	
	vector<Block> block_list = split_driver(argv[1], cloud, cloud_normals, min, max, length, width, height, resolution, argv[2]);
	
	/* Fitting GMM */
	
	//gmm_train();
	
	//vector<Block> final_list = prune_tree(tree);
	
	
	//cerr<<"No. blocks before merge "<<tree.size()<<"\n";
	
	//merge_Block(block_list);
	
	//cerr<<"No. blocks after merge"<<tree.size()<<"\n";
	
	initialise_parameters(resolution, atoi(argv[3]), block_list, length, width, height, min, max);
	
	draw_main(argc, argv);	
	
	
	/*for(int i = 0; i < block_list.size(); i++){
		cout<<(double)block_list[i].height/(double)block_list[i].length<<", "<<(double)block_list[i].height/(double)block_list[i].width<<", "<<(double)block_list[i].width/(double)block_list[i].length<<"\n";
	}*/
		
	return 0;


}	

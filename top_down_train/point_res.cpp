#include <vector>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d_omp.h>

#ifndef RES_HEADER
#define RES_HEADER

double computeCloudResolution (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud)
{
  double res = 0.0;
  int n_points = 0;
  int nres;
  std::vector<int> indices (2);
  std::vector<float> sqr_distances (2);
  pcl::search::KdTree<pcl::PointXYZ> tree;
  tree.setInputCloud (cloud);

  for (size_t i = 0; i < cloud->size (); ++i)
  {
    if (! pcl_isfinite ((*cloud)[i].x))
    {
      continue;
    }
    //Considering the second neighbor since the first is the point itself.
    nres = tree.nearestKSearch (i, 2, indices, sqr_distances);
    if (nres == 2)
    {
      res += sqrt (sqr_distances[1]);
      ++n_points;
    }
  }
  if (n_points != 0)
  {
    res /= n_points;
  }
  return res;
}

void compute_normal(pcl::PointCloud<pcl::PointXYZRGB>::Ptr model, pcl::PointCloud<pcl::Normal>::Ptr &model_normals)
{	
  model_normals = pcl::PointCloud<pcl::Normal>::Ptr( new pcl::PointCloud<pcl::Normal> () );
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr m_xyz( new pcl::PointCloud<pcl::PointXYZ> () );
  pcl::copyPointCloud(*model, *m_xyz);
  
  pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> norm_est;
  norm_est.setKSearch (10);
  norm_est.setInputCloud (m_xyz);
  norm_est.compute (*model_normals);

  cerr << "Normals calculated. ";
  
 /* for(int i = 0; i < model_normals->size(); i++){
	cerr<<model_normals->points[i].normal_x<<" "<<model_normals->points[i].normal_y<<" "<<model_normals->points[i].normal_z<<"\n";
  }	
*/

}


#endif

// Service node to take a PointCloud2 and find the most likely plane in it.

#include <ros/ros.h>

#include <magic_tabletop/FitPlane.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>

#include <pcl_conversions/pcl_conversions.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>


using namespace ros;
using namespace pcl;


bool fit_plane(magic_tabletop::FitPlane::Request &request, magic_tabletop::FitPlane::Response &response) {
  // Translate the point cloud
  PointCloud<PointXYZ>::Ptr cloud (new pcl::PointCloud<PointXYZ>);
  pcl::fromROSMsg(request.cloud, *cloud);

  // All the objects needed
  pcl::PassThrough<PointXYZ> pass;
  pcl::NormalEstimation<PointXYZ, pcl::Normal> ne;
  pcl::SACSegmentation<PointXYZ> seg;
  pcl::ExtractIndices<PointXYZ> extract;
  pcl::ExtractIndices<pcl::Normal> extract_normals;
  
  // Datasets
  pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients), coefficients_cylinder (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices), inliers_cylinder (new pcl::PointIndices);

  // Create the segmentation object for the planar model and set all the parameters
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);//NORMAL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (100);
  seg.setDistanceThreshold (0.05);
  seg.setInputCloud (cloud);

  // Obtain the plane inliers and coefficients
  seg.segment (*inliers_plane, *coefficients_plane);
  //std::cerr << "Plane coefficients: " << *coefficients_plane << std::endl;
  
  /*
  // Extract the planar inliers from the input cloud
  extract.setInputCloud (cloud_filtered);
  extract.setIndices (inliers_plane);
  extract.setNegative (false);
  
  // Write the planar inliers to disk
  pcl::PointCloud<PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<PointXYZ> ());
  extract.filter (*cloud_plane);
  std::cerr << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

  // Remove the planar inliers, extract the rest
  extract.setNegative (true);
  extract.filter (*cloud_filtered);
  extract_normals.setNegative (true);
  extract_normals.setInputCloud (cloud_normals);
  extract_normals.setIndices (inliers_plane);
  extract_normals.filter (*cloud_normals);
  */

  response.a = coefficients_plane->values[0];
  response.b = coefficients_plane->values[1];
  response.c = coefficients_plane->values[2];
  response.d = coefficients_plane->values[3];

  return true;
}


int main(int argc, char **argv) {
  init(argc, argv, "fit_plane");

  NodeHandle node;

  // Set up the service
  ServiceServer service = node.advertiseService("fit_plane", fit_plane);

  spin();
}

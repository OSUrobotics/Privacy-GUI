// Service node to take a PointCloud2 and filter by X-, Y-, and Z-axes.

#include <ros/ros.h>

#include <magic_tabletop/FilterCloudByAxes.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>

#include <pcl_conversions/pcl_conversions.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>

using namespace ros;
using namespace pcl;


bool filter_cloud(magic_tabletop::FilterCloudByAxes::Request &request, magic_tabletop::FilterCloudByAxes::Response &response) {
  // Translate the point cloud
  PointCloud<PointXYZRGB>::Ptr cloud (new pcl::PointCloud<PointXYZRGB>);
  fromROSMsg(request.cloud_in, *cloud);

  ROS_INFO("Before filtering: %i points", cloud->points.size());

  // All the objects needed
  pcl::PassThrough<PointXYZRGB> pass;
  
  // Datasets
  pcl::PointCloud<PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<PointXYZRGB>);

  // Hack
  pcl::PointCloud<PointXYZRGB>::Ptr cloud_filtered1 (new pcl::PointCloud<PointXYZRGB>);
  pcl::PointCloud<PointXYZRGB>::Ptr cloud_filtered2 (new pcl::PointCloud<PointXYZRGB>);

  // Build a passthrough filter to remove spurious NaNs
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (request.limits_z[0], request.limits_z[1]);
  pass.filter (*cloud_filtered1);
  
  pass.setInputCloud (cloud_filtered1);
  pass.setFilterFieldName ("x");
  pass.setFilterLimits (request.limits_x[0], request.limits_x[1]);
  pass.filter (*cloud_filtered2);

  pass.setInputCloud (cloud_filtered2);
  pass.setFilterFieldName ("y");
  pass.setFilterLimits (request.limits_y[0], request.limits_y[1]);
  pass.filter (*cloud_filtered);

  ROS_INFO("After filtering: %i points", cloud_filtered->points.size());

  toROSMsg(*cloud_filtered, response.cloud_out);

  toROSMsg(response.cloud_out, response.image_out);

  return true;
}


int main(int argc, char **argv) {
  init(argc, argv, "filter_cloud_by_axes");

  NodeHandle node;

  // Set up the service
  ServiceServer service = node.advertiseService("filter_cloud_by_axes", filter_cloud);

  spin();
}

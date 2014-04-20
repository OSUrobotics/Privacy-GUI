// Service node to take a PointCloud2 and transform it to another TF frame.

#include <ros/ros.h>

#include <tf/transform_listener.h>

#include <magic_tabletop/TransformPointCloud2.h>

#include <pcl_ros/transforms.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>

#include <pcl_conversions/pcl_conversions.h>

using namespace ros;
using namespace pcl;


bool transform_cloud(magic_tabletop::TransformPointCloud2::Request &request, magic_tabletop::TransformPointCloud2::Response &response) {
  
  tf::TransformListener listener;

  bool can_transform;

  can_transform = listener.waitForTransform(request.cloud_in.header.frame_id, request.frame_new,
					    ros::Time(0), ros::Duration(3.0));

  if (can_transform){

    request.cloud_in.header.stamp = ros::Time(0);
    pcl_ros::transformPointCloud(request.frame_new, request.cloud_in, response.cloud_out, listener);

  }

  return true;
}


int main(int argc, char **argv) {
  init(argc, argv, "transform_PointCloud2");

  NodeHandle node;

  // Set up the service
  ServiceServer service = node.advertiseService("transform_PointCloud2", transform_cloud);

  spin();
}

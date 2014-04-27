//
#include <iostream>

// ROS
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float32MultiArray.h>

// PCL
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <pcl_conversions/pcl_conversions.h>

// Iterator stuff
#include <iterator>

using namespace std;
using namespace pcl;
using namespace ros;


void unpack_cloud(const sensor_msgs::PointCloud2ConstPtr& cloud_in)
{
  // Unpack point cloud message into PCL data type
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  fromROSMsg(*cloud_in, *cloud);

  // Iterator for PCL point cloud data structure
  pcl::PointCloud<pcl::PointXYZRGB>::iterator it;

  // Turn all points near the plane to green
  std_msgs::Float32MultiArray x, y, z, r, g, b;

  for ( it = (*cloud).begin(); it != (*cloud).end(); ++it )
    {
      x.data.push_back(it -> x);
      y.data.push_back(it -> y);
      z.data.push_back(it -> z);
      r.data.push_back(it -> r);
      g.data.push_back(it -> g);
      b.data.push_back(it -> b);

    }

  ROS_INFO("Processed a cloud.");

  //toROSMsg(*cloud, response.cloud_out);

  //toROSMsg(response.cloud_out, response.image_out);

  return;
}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "unpack_PointCloud2");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("/camera/depth_registered/points", 1, unpack_cloud);
  
  ros::spin();
  
}

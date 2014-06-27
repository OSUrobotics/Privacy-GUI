//
#include <iostream>

// ROS
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

// PCL
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <pcl_conversions/pcl_conversions.h>

// Iterator stuff
#include <iterator>

// Service definition
#include <magic_tabletop/FillInPlane.h>

using namespace std;
using namespace pcl;
using namespace ros;

bool fill_in_plane(magic_tabletop::FillInPlane::Request &request, magic_tabletop::FillInPlane::Response &response)
{
  // Unpack point cloud message into PCL data type
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  fromROSMsg(request.cloud_in, *cloud);

  // Iterator for PCL point cloud data structure
  pcl::PointCloud<pcl::PointXYZRGB>::iterator it;

  // Turn all points near the plane to green
  unsigned long count = 0;
  float last_x = nanf(""); float last_y = nanf(""); float last_z = nanf("");
  float twoago_x = nanf(""); float twoago_y = nanf(""); float twoago_z = nanf("");

  for ( it = (*cloud).begin(); it != (*cloud).end(); ++it )
    {
      // Choose all points near the plane
      if ((it -> x) > -1 & (it -> x) < 1 & (it -> y) > -1 & (it -> y) < 1 & (it -> z) > request.bottom & (it -> z) < request.top) {
	
	// Try to fill in NaN points
	if (isnan(it -> x) & !isnan(last_x) & !isnan(twoago_x)) {
	  it -> x = last_x + (last_x - twoago_x);
	  it -> y = last_y + (last_y - twoago_y);
	  it -> z = last_z + (last_z - twoago_z);
	  //it -> z = 0;  // snap to plane
	}

	// Change color to GREEN
	if (!isnan(it -> x)) {
	  it -> r = 0;
	  it -> g = 255;
	  it -> b = 0;
	}

	// Keep track of previous points
	twoago_x = last_x; twoago_y = last_y;
	last_x = it -> x; last_y = it -> y;

	count++;
      }

    }

  ROS_INFO("Filled in %lu points.", count);

  toROSMsg(*cloud, response.cloud_out);

  toROSMsg(response.cloud_out, response.image_out);

  return true;
}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "fill_in_plane");
  ros::NodeHandle nh;
  
  ros::ServiceServer service = nh.advertiseService("fill_in_plane", fill_in_plane);
  
  ros::spin();
  
}

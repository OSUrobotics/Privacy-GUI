//
#include <iostream>

// ROS
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float32.h>
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


class CloudUnpacker
{
public:

  ros::Publisher pub;

  //! Constructor.
  CloudUnpacker()
  {
  }

  //! Destructor.
  ~CloudUnpacker()
  {
  }

  void load_publisher(ros::Publisher pub_in)
  {
    pub = pub_in;
  }

  void unpack_cloud(const sensor_msgs::PointCloud2ConstPtr& cloud_in)
  {
    // Unpack point cloud message into PCL data type
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    fromROSMsg(*cloud_in, *cloud);
    
    // Iterator for PCL point cloud data structure
    pcl::PointCloud<pcl::PointXYZRGB>::iterator it;
    
    // Turn all points near the plane to green
    std_msgs::Float32MultiArray xyzrgb;
    
    for ( it = (*cloud).begin(); it != (*cloud).end(); ++it )
      {
	xyzrgb.data.push_back(it -> x);
	xyzrgb.data.push_back(it -> y);
	xyzrgb.data.push_back(it -> z);
	xyzrgb.data.push_back(it -> r);
	xyzrgb.data.push_back(it -> g);
	xyzrgb.data.push_back(it -> b);
      }
    
    ROS_INFO("Processed a cloud.");
    
    pub.publish(xyzrgb);

    return;
  }

};




int main (int argc, char** argv)
{

  // Initialize ROS
  ros::init (argc, argv, "unpack_PointCloud2");
  ros::NodeHandle nh;

  CloudUnpacker *unpacker = new CloudUnpacker();

  // Publisher of unpacked points
  ros::Publisher pub = nh.advertise<std_msgs::Float32MultiArray>("/camera/depth_registered/points_unpacked", 1);

  unpacker->load_publisher(pub);

  // Subscriber to PointCloud2 msg
  ros::Subscriber sub = nh.subscribe("/camera/depth_registered/points", 1, &CloudUnpacker::unpack_cloud, unpacker);

  ros::spin();
  
}

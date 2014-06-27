//
#include <iostream>
#include <string>

// ROS
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

// TF
#include <tf/transform_listener.h>

// PCL
#include <pcl_ros/transforms.h>
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


class CloudHandler
{
public:

  ros::Publisher pub;

  tf::TransformListener listener;
  bool can_transform;
    

  //! Constructor.
  CloudHandler()
  {
  }

  //! Destructor.
  ~CloudHandler()
  {
  }

  void load_publisher(ros::Publisher pub_in)
  {
    pub = pub_in;
  }

  void handle_cloud(const sensor_msgs::PointCloud2ConstPtr& cloud_in)
  {
    // Convert to PCL

    // Transform to AR Tag frame
    sensor_msgs::PointCloud2 cloud_tf, cloud_out;
    string frame_new = "/ar_marker_3";
    cloud_tf = transform_cloud(*cloud_in, frame_new);
    cloud_out = filter_cloud(cloud_tf, -0.10, 10);

    // Turn all points above the plane to black
    // Convert to ROS Image msg
    sensor_msgs::Image image_out;
    toROSMsg(cloud_out, image_out);

    // Publish out!
    pub.publish(image_out);
  }

  sensor_msgs::PointCloud2 transform_cloud(sensor_msgs::PointCloud2 cloud_in, string frame_new)
  {
    can_transform = listener.waitForTransform(cloud_in.header.frame_id, frame_new,
					      ros::Time(0), ros::Duration(3.0));

    if (can_transform){
      sensor_msgs::PointCloud2 cloud_out;
      cloud_in.header.stamp = ros::Time(0);
      pcl_ros::transformPointCloud(frame_new, cloud_in, cloud_out, listener);
      return cloud_out;
    }
    else {
      ROS_ERROR("Could not transform PointCloud2!");
    }
  }

  sensor_msgs::PointCloud2 filter_cloud(sensor_msgs::PointCloud2 cloud_in, float lower, float upper)
  {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    fromROSMsg(cloud_in, *cloud);
    
    // Iterator for PCL point cloud data structure
    pcl::PointCloud<pcl::PointXYZRGB>::iterator it;
    
    // Turn all points near the plane to BLACK
    unsigned long count = 0;

    for ( it = (*cloud).begin(); it != (*cloud).end(); ++it )
      {
	// Choose all points near the plane
	if (((it -> z) > lower & (it -> z) < upper)) {// | isnan(it -> x)) {
	  
	  // Change color to BLACK
	  it -> r = 0;
	  it -> g = 0;
	  it -> b = 0;

	  count++;
	}

      }
    
    ROS_INFO("Filled in %lu points.", count);
    
    sensor_msgs::PointCloud2 cloud_out;
    toROSMsg(*cloud, cloud_out);

    return cloud_out;
  }

};


int main (int argc, char** argv)
{

  // Initialize ROS
  ros::init (argc, argv, "redact_plane");
  ros::NodeHandle nh;

  CloudHandler *handler = new CloudHandler();

  // Publisher of filtered image
  ros::Publisher pub = nh.advertise<sensor_msgs::Image>("/camera/rgb/image_filtered", 1);

  handler->load_publisher(pub);

  // Subscriber to PointCloud2 msg
  ros::Subscriber sub = nh.subscribe("/camera/depth_registered/points", 1, &CloudHandler::handle_cloud, handler);

  ros::spin();
  
}

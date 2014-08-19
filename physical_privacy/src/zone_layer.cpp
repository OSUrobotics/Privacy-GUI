#include "zone_layer.h"
#include <pluginlib/class_list_macros.h>
#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "std_msgs/Bool.h"
#include "opencv2/opencv.hpp"

// Register ZoneLayer class as a plugin
PLUGINLIB_EXPORT_CLASS(zone_layer_namespace::ZoneLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::NO_INFORMATION;

namespace zone_layer_namespace
{

ZoneLayer::ZoneLayer() {}

// Called upon loading costmap layers (at stat of move_base)
void ZoneLayer::onInitialize()
{
	ros::NodeHandle nh("~/" + name_), g_nh;
	ROS_INFO("Activating Zone Layer.");

	// init class vars
	zones_received_ = false;
	current_ = true;
	default_value_ = 0;
	matchSize();
	cost_img_ = cv::Mat(getSizeInCellsX(), getSizeInCellsY(), CV_8U);

	// I don't get it either, but I'm pretty sure it's probably important
	dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
	dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
			&ZoneLayer::reconfigureCB, this, _1, _2);
	dsrv_->setCallback(cb);

	// Advertise /restrict_zones service 
	service_ = g_nh.advertiseService("restrict_zones", &zone_layer_namespace::ZoneLayer::zones, this);

}

// callback for /restrict_zones
bool ZoneLayer::zones(physical_privacy::restrictZones::Request  &req, 
					physical_privacy::restrictZones::Response &res) 
{
	// erase old zones
	cost_img_ = cv::Scalar(0);

	// declare args to cv::fillPoly
	const cv::Point **pts;
	int n_polygons = req.polygons.size();
	pts = new const cv::Point* [n_polygons];
	int *polygon_sizes = new int [n_polygons];

	// turn the service request into an array of arrays of cv::Points 
	for (int i = 0; i < n_polygons; i++) 
	{
		// cv needs the sizes of the each polygon
		polygon_sizes[i] = req.polygons[i].points.size();

		// the points in the polygon i
		cv::Point *tmp = new cv::Point[polygon_sizes[i]];
		for (int j = 0; j < polygon_sizes[i]; j++) {
			// find real world coordinates in image coordinates
			double wx, wy;
			wx = req.polygons[i].points[j].x;
			wy = req.polygons[i].points[j].y;
			unsigned int mx, my;
			res.success.data = worldToMap(wx, wy, mx, my);
			tmp[j] = cv::Point(mx, my);

			// Stop if you find a point out of bounds
			if (!res.success.data) {
				img_to_map_();
				zones_received_ = true;
				return true;
			}
		}

		// point the polygon list to the newest one
		// Had to do it this way because cv is WEIRD
		pts[i] = tmp;
	} 

	// Finally, the function call!
	cv::fillPoly(cost_img_, pts, polygon_sizes, n_polygons, cv::Scalar(LETHAL_OBSTACLE));

	// clean up
	for (int a = 0; a < n_polygons; a++) {
		delete [] pts[a];
	} 
	delete [] pts;
	delete [] polygon_sizes;

	// Copy cost image into costmap layer
	img_to_map_();
	
	// Yep, we received a zone
	zones_received_ = true;
	return true;
}

// Copy cost image directly into costamp layer
void ZoneLayer::img_to_map_() {
	for (int i = 0; i < getSizeInCellsX(); i++) {
		for (int j = 0; j < getSizeInCellsY(); j++) {
			unsigned char cost = cost_img_.at<unsigned char>(j, i);
			setCost(i, j, cost);
		}
	}
}

// make this costmap layer match the size of the master
void ZoneLayer::matchSize()
{
	Costmap2D* master = layered_costmap_->getCostmap();
	resizeMap(master->getSizeInCellsX(), 
				master->getSizeInCellsY(), 
				master->getResolution(),
				master->getOriginX(), 
				master->getOriginY());
}

// Magic black box don't ask me
void ZoneLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
{
	enabled_ = config.enabled;

}

// Only update things when the callback has FINISHED
void ZoneLayer::updateBounds(double origin_x, 
								double origin_y, 
								double origin_yaw, 
								double* min_x,
								double* min_y, 
								double* max_x, 
								double* max_y)
{
	// Did I get any callbacks while I was gone?
	ros::spinOnce();
	
	// Update everything iff we got a callback
	if ( enabled_ && zones_received_) {
		double wx, wy;

		mapToWorld(0, 0, wx, wy);
		*min_x = wx;
		*min_y = wy;

		mapToWorld(getSizeInCellsX(), getSizeInCellsY(), wx, wy);
		*max_x = wx;
		*max_y = wy;

		// Okay I'm finished with that callback
		zones_received_ = false;

	}
}

// Actually update costs in the master grid
void ZoneLayer::updateCosts(costmap_2d::Costmap2D& master_grid, 
								int min_i, 
								int min_j, 
								int max_i,
								int max_j)
{
	if (enabled_) {
		for (int j = min_j; j < max_j; j++)
		{
			for (int i = min_i; i < max_i; i++)
			{
				// if our layer has a cost, set the master layer to it.
				int cost = getCost(i, j);
				if (cost) {
					master_grid.setCost(i, j, cost);
				}
			}
		}
	}

}

} // end namespace
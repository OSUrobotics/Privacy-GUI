#include "zone_layer.h"
#include <pluginlib/class_list_macros.h>
#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "std_msgs/Bool.h"
#include "opencv2/opencv.hpp"


PLUGINLIB_EXPORT_CLASS(zone_layer_namespace::ZoneLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::NO_INFORMATION;

namespace zone_layer_namespace
{

ZoneLayer::ZoneLayer() {}


void ZoneLayer::onInitialize()
{
	ros::NodeHandle nh("~/" + name_), g_nh;

	ROS_INFO("Activating Zone Layer.");

	zones_received_ = false;
	current_ = true;
	default_value_ = 0;
	matchSize();

	dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
	dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
			&ZoneLayer::reconfigureCB, this, _1, _2);
	dsrv_->setCallback(cb);

	cost_img_ = cv::Mat(getSizeInCellsX(), getSizeInCellsY(), CV_8U);

	// ROS_INFO("NO_INFORMATION = %d, LETHAL_OBSTACLE = %d", NO_INFORMATION, LETHAL_OBSTACLE);

	service_ = g_nh.advertiseService("restrict_zones", &zone_layer_namespace::ZoneLayer::zones, this);
	// ros::spinOnce();

	// zones();
}

bool ZoneLayer::zones(physical_privacy::restrictZones::Request  &req, 
					physical_privacy::restrictZones::Response &res) 
{
	cost_img_ = cv::Scalar(0);
	// img_to_map_();

	const cv::Point **pts;
	int n_polygons = req.polygons.size();
	pts = new const cv::Point* [n_polygons];
	int *polygon_sizes = new int [n_polygons];

	std::cout << "Polygons:" << std::endl;
	for (int i = 0; i < n_polygons; i++) 
	{
		std::cout << "\tPolygon " << i << std::endl;
		polygon_sizes[i] = req.polygons[i].points.size();
		// polygon_sizes[0] = 3;
		cv::Point *tmp = new cv::Point[polygon_sizes[i]];
		for (int j = 0; j < polygon_sizes[i]; j++) {
			double wx, wy;
			wx = req.polygons[i].points[j].x;
			wy = req.polygons[i].points[j].y;
			unsigned int mx, my;
			res.success.data = worldToMap(wx, wy, mx, my);
			if (!res.success.data) {
				img_to_map_();
				zones_received_ = true;
				return true;
			}
			std::cout << "\t\tPoint " << mx << ", "<< my << std::endl;
			tmp[j] = cv::Point(mx, my);
		}
		pts[i] = tmp;
	} 

	cv::fillPoly(cost_img_, pts, polygon_sizes, n_polygons, cv::Scalar(LETHAL_OBSTACLE));

	// std::cout << "Once again, Polygons:" << std::endl;
	for (int a = 0; a < n_polygons; a++) {
		delete [] pts[a];
	} 
	delete [] pts;
	delete [] polygon_sizes;

	img_to_map_();
	
	zones_received_ = true;
	return true;
}

void ZoneLayer::img_to_map_() {
	for (int i = 0; i < getSizeInCellsX(); i++) {
		for (int j = 0; j < getSizeInCellsY(); j++) {
			unsigned char cost = cost_img_.at<unsigned char>(j, i);
			setCost(i, j, cost);
		}
	}
}

void ZoneLayer::matchSize()
{
	Costmap2D* master = layered_costmap_->getCostmap();
	resizeMap(master->getSizeInCellsX(), 
				master->getSizeInCellsY(), 
				master->getResolution(),
				master->getOriginX(), 
				master->getOriginY());
}


void ZoneLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
{
	enabled_ = config.enabled;

}

void ZoneLayer::updateBounds(double origin_x, 
								double origin_y, 
								double origin_yaw, 
								double* min_x,
								double* min_y, 
								double* max_x, 
								double* max_y)
{
	ros::spinOnce();
	
	if ( enabled_ && zones_received_) {
		double wx, wy;

		mapToWorld(0, 0, wx, wy);
		*min_x = wx;
		*min_y = wy;

		mapToWorld(getSizeInCellsX(), getSizeInCellsY(), wx, wy);
		*max_x = wx;
		*max_y = wy;


		zones_received_ = false;

	}
	// std::cout << "Updatin' bounds to: (" << *min_x << ", " << *min_y << ") (" << *max_x << ", " << *max_y << ")" << std::endl;
}

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
				int cost = getCost(i, j);
				if (cost) {
					// std::cout << i << ", " << j << std::endl;
					master_grid.setCost(i, j, cost);
				}
			}
		}
	}

}

} // end namespace
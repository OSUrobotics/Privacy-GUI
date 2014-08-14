#include "zone_layer.h"
#include <pluginlib/class_list_macros.h>
#include <iostream>


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

	current_ = true;
	default_value_ = NO_INFORMATION;
	matchSize();

	dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
	dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
			&ZoneLayer::reconfigureCB, this, _1, _2);
	dsrv_->setCallback(cb);



	load();
}

void ZoneLayer::load() 
{
	// nav_msgs::GetMap srv_;

	// client_.waitForExistence();
	// if (client_.call(srv_)) {
	// 	std::cout << srv_.response.map.info;
	// }
	// else {

	// 	ROS_ERROR("SERVICE CALL FAILED");
	// 	exit(1);
	// }

	// for (int i = 0; i < getSizeInCellsX() * getSizeInCellsY(); i++)
	// {
	// 	costmap_[i] = (srv_.response.map.data[i]);
	// }
}

void ZoneLayer::matchSize()
{
	Costmap2D* master = layered_costmap_->getCostmap();
	resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(),
				master->getOriginX(), master->getOriginY());
}


void ZoneLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
{
	enabled_ = config.enabled;

}

void ZoneLayer::updateBounds(double origin_x, double origin_y, double origin_yaw, double* min_x,
									double* min_y, double* max_x, double* max_y)
{
	// if (enabled_ == was_enabled_) {
	// 	return;
	// }
	// else {
	double wx, wy;

	mapToWorld(0, 0, wx, wy);
	*min_x = wx;
	*min_y = wy;

	mapToWorld(getSizeInCellsX(), getSizeInCellsY(), wx, wy);
	*max_x = wx;
	*max_y = wy;

	// }
}

void ZoneLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i,
																					int max_j)
{
	if (enabled_) {
		for (int j = min_j; j < max_j; j++)
		{
			for (int i = min_i; i < max_i; i++)
			{
				int index = getIndex(i, j);
				int cost = master_grid.getCost(i, j); 
				if (cost >= 50)
					continue;
				master_grid.setCost(i, j, costmap_[index]);
			}
		}
	}

}

} // end namespace
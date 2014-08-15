#ifndef ZONE_LAYER_H_
#define ZONE_LAYER_H_
// ROS Libraries
#include <ros/ros.h>
#include <costmap_2d/costmap_layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <dynamic_reconfigure/server.h>
// msgs and srvs
#include "physical_privacy/restrictZones.h"
// external libraries
#include "opencv2/core/core.hpp"

namespace zone_layer_namespace
{

	class ZoneLayer : public costmap_2d::CostmapLayer
	{
	public:

		ZoneLayer();

		virtual void onInitialize();

		virtual void updateBounds(double origin_x, 
									double origin_y, 
									double origin_yaw, 
									double* min_x, 
									double* min_y, 
									double* max_x,
									double* max_y);

		virtual void updateCosts(costmap_2d::Costmap2D& master_grid, 
									int min_i, 
									int min_j, 
									int max_i, 
									int max_j);

		bool isDiscretized() { return true; }

		virtual void matchSize();


	private:
		void reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level);

		bool zones(physical_privacy::restrictZones::Request  &req, 
					physical_privacy::restrictZones::Response &res); 

		void img_to_map_();

		dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig> *dsrv_;

		cv::Mat cost_img_;

		ros::ServiceServer service_;

		bool zones_received_;

	};
}
#endif
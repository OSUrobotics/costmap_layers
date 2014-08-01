#include "learned_layer.h"
#include <pluginlib/class_list_macros.h>
#include <iostream>


PLUGINLIB_EXPORT_CLASS(simple_layer_namespace::LearnedLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::NO_INFORMATION;

namespace simple_layer_namespace
{

LearnedLayer::LearnedLayer() {}

void LearnedLayer::onInitialize()
{
	ros::NodeHandle nh("~/" + name_), g_nh;
	client_ = g_nh.serviceClient<nav_msgs::GetMap>("costmap_server");
	ROS_INFO("Activating Learned Layer");

	was_enabled_ = false;
	current_ = true;
	default_value_ = NO_INFORMATION;
	matchSize();

	dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
	dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
			&LearnedLayer::reconfigureCB, this, _1, _2);
	dsrv_->setCallback(cb);

	load();
}

void LearnedLayer::load() 
{
	nav_msgs::GetMap srv_;

	client_.waitForExistence();
	if (client_.call(srv_)) {
		std::cout << srv_.response.map.info;
	}
	else {

		ROS_ERROR("SERVICE CALL FAILED");
		exit(1);
	}

	for (int i = 0; i < getSizeInCellsX() * getSizeInCellsY(); i++)
	{
		costmap_[i] = (100 - srv_.response.map.data[i]) / 2;
	}
}

void LearnedLayer::matchSize()
{
	Costmap2D* master = layered_costmap_->getCostmap();
	resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(),
				master->getOriginX(), master->getOriginY());
}


void LearnedLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
{
	enabled_ = config.enabled;
}

void LearnedLayer::updateBounds(double origin_x, double origin_y, double origin_yaw, double* min_x,
									double* min_y, double* max_x, double* max_y)
{
	if (enabled_ == was_enabled_) {
		return;
	}
	else {
		double wx, wy;

		mapToWorld(0, 0, wx, wy);
		*min_x = wx;
		*min_y = wy;

		mapToWorld(getSizeInCellsX(), getSizeInCellsY(), wx, wy);
		*max_x = wx;
		*max_y = wy;

	}
}

void LearnedLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i,
																					int max_j)
{


	if (enabled_) {
		for (int j = min_j; j < max_j; j++)
		{
			for (int i = min_i; i < max_i; i++)
			{
				int index = getIndex(i, j);
				if (master_grid.getCost(i, j) >= 50)
					continue;
				master_grid.setCost(i, j, costmap_[index]);
			}
		}
	}

	was_enabled_ = enabled_;
}

} // end namespace
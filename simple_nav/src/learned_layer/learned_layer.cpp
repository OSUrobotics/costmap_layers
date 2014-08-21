#include "learned_layer.h"
#include <pluginlib/class_list_macros.h>
#include <iostream>
#include <vector>
#include <boost/foreach.hpp>
#include <math.h>



PLUGINLIB_EXPORT_CLASS(simple_layer_namespace::LearnedLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::NO_INFORMATION;

namespace simple_layer_namespace
{

LearnedLayer::LearnedLayer() {}

void LearnedLayer::onInitialize()
{
	ros::NodeHandle nh("~/" + name_), g_nh;
	if (! g_nh.hasParam("costmap_file")) {
		std::cout << name_;
		ROS_ERROR("Couldn't find param /costmap_file");
		exit(1);
	}
	g_nh.getParam("costmap_file", costmap_path_);
	ROS_INFO("Activating Learned Layer with file %s", costmap_path_.c_str());

	was_enabled_ = false;
	current_ = true;
	// because GridCells messages are restricted to [0, 100]
	default_value_ = 100;
	matchSize();

	dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
	dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
			&LearnedLayer::reconfigureCB, this, _1, _2);
	dsrv_->setCallback(cb);

	load();
}

void LearnedLayer::load() 
{
	rosbag::Bag bag;

	bag.open(costmap_path_, rosbag::bagmode::Read);

	std::vector<std::string> topics;
    topics.push_back(std::string("costmap"));

    rosbag::View view(bag, rosbag::TopicQuery(topics));

    nav_msgs::OccupancyGrid::ConstPtr map_grid;

    BOOST_FOREACH(rosbag::MessageInstance const m, view)
    {
        map_grid = m.instantiate<nav_msgs::OccupancyGrid>();
        if (map_grid == NULL) {
            ROS_ERROR("Failed to read bag file");
        }

    }

    for (int i = 0; i < getSizeInCellsX() * getSizeInCellsY(); i++)
	{
		double c = (100 - map_grid->data[i])/ 100.0;
		c = pow(0.1, c);
		unsigned char cost = (100 * c);
		costmap_[i] = cost;
	}

    bag.close();
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
		// for (int j = min_j; j < max_j; j++)
		// {
		// 	for (int i = min_i; i < max_i; i++)
		// 	{
		// 		int index = getIndex(i, j);
		// 		int cost = master_grid.getCost(i, j); 
		// 		if (cost >= 50)
		// 			continue;
		// 		master_grid.setCost(i, j, costmap_[index]);
		// 	}
		// }
		updateWithMax(master_grid, min_i, min_j, max_i, max_j);
	}

	was_enabled_ = enabled_;
}

} // end namespace
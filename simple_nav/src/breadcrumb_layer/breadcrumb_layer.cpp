#include "breadcrumb_layer.h"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(simple_layer_namespace::BreadcrumbLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::NO_INFORMATION;

namespace simple_layer_namespace
{

BreadcrumbLayer::BreadcrumbLayer() {}

void BreadcrumbLayer::onInitialize()
{
	ros::NodeHandle nh("~/" + name_);
	current_ = true;
	default_value_ = 50;
	matchSize();

	dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
	dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
			&BreadcrumbLayer::reconfigureCB, this, _1, _2);
	dsrv_->setCallback(cb);

}


void BreadcrumbLayer::matchSize()
{
	Costmap2D* master = layered_costmap_->getCostmap();
	resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(),
				master->getOriginX(), master->getOriginY());
}


void BreadcrumbLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
{
	enabled_ = config.enabled;
}

void BreadcrumbLayer::updateBounds(double origin_x, double origin_y, double origin_yaw, double* min_x,
									double* min_y, double* max_x, double* max_y)
{
	if (!enabled_)
		return;

	double mark_x = origin_x + cos(origin_yaw), mark_y = origin_y + sin(origin_yaw);
	unsigned int mx;
	unsigned int my;
	// if(worldToMap(mark_x, mark_y, mx, my)){
	// 	setCost(mx, my, 5);
	// }
	if(worldToMap(origin_x, origin_y, mx, my)){
		setCost(mx, my, 0);
	}
	
	*min_x = std::min(*min_x, mark_x);
	*min_y = std::min(*min_y, mark_y);
	*max_x = std::max(*max_x, mark_x);
	*max_y = std::max(*max_y, mark_y);

	*min_x = std::min(*min_x, origin_x);
	*min_y = std::min(*min_y, origin_y);
	*max_x = std::max(*max_x, origin_x);
	*max_y = std::max(*max_y, origin_y);
}

void BreadcrumbLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i,
																					int max_j)
{
	if (!enabled_)
		return;

	for (int j = min_j; j < max_j; j++)
	{
		for (int i = min_i; i < max_i; i++)
		{
			int index = getIndex(i, j);
			if (master_grid.getCost(i, j) >= 100)
				continue;
			master_grid.setCost(i, j, costmap_[index]);
		}
	}
}

} // end namespace
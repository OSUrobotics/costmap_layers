#include <pluginlib/class_loader.h>
#include "simple_layer.h"

int main(int argc, char** argv)
{
  pluginlib::ClassLoader<costmap_2d::Layer> costmap_loader("simple_nav", "costmap_2d::Layer");

  try
  {
    boost::shared_ptr<costmap_2d::Layer> triangle = costmap_loader.createInstance("simple_layer_namespace::SimpleLayer");

    ROS_INFO("It worked!");
  }
  catch(pluginlib::PluginlibException& ex)
  {
    ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
  }

  return 0;
}
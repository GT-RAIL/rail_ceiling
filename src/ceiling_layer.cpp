#include <rail_ceiling/ceiling_layer.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(ceiling_layer_namespace::CeilingLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::NO_INFORMATION;

namespace ceiling_layer_namespace
{

CeilingLayer::CeilingLayer()
{
}

void CeilingLayer::onInitialize()
{
  ros::NodeHandle nh("~/" + name_);
  mapReceived = false;
  current_ = true;
  default_value_ = NO_INFORMATION;
  matchSize();

  dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(&CeilingLayer::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);

  nh.param<std::string>("map_topic", map_topic, "/markers_to_map/ar_global_cost_map");

  map_in = nh.subscribe < nav_msgs::OccupancyGrid > (map_topic, 1, &CeilingLayer::map_in_cback, this);
}

void CeilingLayer::matchSize()
{
  Costmap2D* master = layered_costmap_->getCostmap();
  resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(), master->getOriginX(), master->getOriginY());
}

void CeilingLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
{
  enabled_ = config.enabled;
}

void CeilingLayer::updateBounds(double origin_x, double origin_y, double origin_yaw, double* min_x, double* min_y, double* max_x, double* max_y)
{
  if (!enabled_ || !mapReceived)
    return;

  *min_x = -std::fabs(obstacleMap.info.origin.position.x);
  *min_y = -std::fabs(obstacleMap.info.origin.position.y);
  *max_x = *min_x + obstacleMap.info.width;
  *max_y = *min_y + obstacleMap.info.height;
}

void CeilingLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_ || !mapReceived)
    return;

  for (int j = min_j; j < max_j; j ++)
  {
    for (int i = min_i; i < max_i; i ++)
    {
      int index = getIndex(i, j);
      if (obstacleMap.data[index] > 1)
      {
	      master_grid.setCost(i, j, LETHAL_OBSTACLE);
      }
    }
  }
}

void CeilingLayer::map_in_cback(const nav_msgs::OccupancyGrid::ConstPtr& map)
{
  obstacleMap = *map;
  mapReceived = true;
  ROS_INFO("Map Received");
}

} // end namespace

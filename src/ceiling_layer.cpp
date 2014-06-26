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
  current_ = true;
  default_value_ = NO_INFORMATION;
  matchSize();

  dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
      &CeilingLayer::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);

  map_in = nh.subscribe < nav_msgs::OccupancyGrid
      > ("/markers_to_map/marker_map", 1, &CeilingLayer::map_in_cback, this);
}

void CeilingLayer::matchSize()
{
  Costmap2D* master = layered_costmap_->getCostmap();
  resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(), master->getOriginX(),
            master->getOriginY());
}

void CeilingLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
{
  enabled_ = config.enabled;
}

void CeilingLayer::updateBounds(double origin_x, double origin_y, double origin_yaw, double* min_x, double* min_y,
                                double* max_x, double* max_y)
{
  if (!enabled_ || !mapReceived)
    return;

  //TODO: this algorithm is bad and I should feel bad (probably)
  *min_x = -std::abs(obstacleMap.info.origin.position.x);
  *min_y = -std::abs(obstacleMap.info.origin.position.y);                                                             
  *max_x = -std::abs(obstacleMap.info.origin.position.x)+obstacleMap.info.width;
  *max_y = -std::abs(obstacleMap.info.origin.position.y)+obstacleMap.info.height;
  //ROS_ERROR("%f, %f, %f, %f",*min_x, *min_y, *max_x, * max_y);

  /*
  double mark_x = origin_x + cos(origin_yaw), mark_y = origin_y + sin(origin_yaw);
  unsigned int mx;
  unsigned int my;
  if (worldToMap(mark_x, mark_y, mx, my))
  {
    setCost(mx, my, LETHAL_OBSTACLE);
  }
  */

  /*
  *min_x = std::min(*min_x, mark_x);
  *min_y = std::min(*min_y, mark_y);
  *max_x = std::max(*max_x, mark_x);
  *max_y = std::max(*max_y, mark_y);
  *max_y */
}

void CeilingLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_ || !mapReceived)
    return;

  for (int j = min_j; j < max_j; j++)
  {
    for (int i = min_i; i < max_i; i++)
    {

      int index = getIndex(i, j);
      if (obstacleMap.data[index] > 1) {
	master_grid.setCost(i, j, 100);
	  //	master_grid.setCost(i, j, LETHAL_OBSTACLE);
	  //	ROS_ERROR("yup");
      } else {
	//        master_grid.setCost(i, j, NO_INFORMATION);
        master_grid.setCost(i, j, 0);
	//	ROS_ERROR("nope");
      }
      /*
      if (costmap_[index] == NO_INFORMATION)
        continue;
      master_grid.setCost(i, j, costmap_[index]);
      */
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

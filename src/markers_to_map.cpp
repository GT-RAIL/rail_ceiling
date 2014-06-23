//TODO FIX comments

/*!
 * \odom_covariance_converter.cpp
 * \brief Adds covariance matrix to odometry message
 *
 * odom_covariance_converter adds a covariance matrix to odometry messages so they are compatible with robot_pose_efk.
 *
 * \author Steven Kordell, WPI - spkordell@wpi.edu
 * \date June 16, 2014
 */

#include <rail_ceiling/markers_to_map.h>

using namespace std;

markers_to_map::markers_to_map()
{
  //a private handle for this ROS node
  ros::NodeHandle node("~");

  // create the ROS topics
  markers_in = node.subscribe<ar_track_alvar::AlvarMarkers>("ar_pose_marker", 10, &markers_to_map::markers_cback, this);
  map_out = node.advertise<nav_msgs::OccupancyGrid>("marker_map", 1);

  ROS_INFO("Markers To Map Started");
}

void markers_to_map::markers_cback(const ar_track_alvar::AlvarMarkers::ConstPtr& marker)
{
  //TODO:
  //Save marker to list of markers
  //update map to represent obstacles found in marker list
  //publish occupancy grid


  nav_msgs::OccupancyGrid map;

  //TODO: finish header
  map.header.frame_id = "ar_map";

  //TODO: parameterize these or get them from the existing map
  map.info.resolution = 0.025;
  map.info.width = 338;
  map.info.height = 427;
  map.info.origin.position.x = -4.25;
  map.info.origin.position.y = -5.25;
  map.info.origin.position.z = 0.0;
  map.info.origin.orientation.x = 00.;
  map.info.origin.orientation.x = 0.0;
  map.info.origin.orientation.z = 0.0;
  map.info.origin.orientation.w = 1.0;


  //vector<signed char, std::allocator<signed char> >;

  vector<signed char> mapData(map.info.width * map.info.height);

  for(int i=0; i<(map.info.width * map.info.height); i++){
    mapData[i] = -1;
  }
  map.data = mapData;

  map_out.publish(map);

}

int main(int argc, char **argv)
{
  // initialize ROS and the node
  ros::init(argc, argv, "markers_to_map");

  // initialize the converter
  markers_to_map converter;

  ros::spin();

  return EXIT_SUCCESS;
}

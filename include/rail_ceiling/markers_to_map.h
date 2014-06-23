//TODO FIX comments


/*!
 * \odom_covariance_converter.h
 * \brief Adds covariance matrix to odometry message
 *
 * odom_covariance_converter adds a covariance matrix to odometry messages so they are compatible with robot_pose_efk.
 *
 * \author Steven Kordell, WPI - spkordell@wpi.edu
 * \date June 16, 2014
 */

#ifndef MARKERS_TO_MAP_H_
#define MARKERS_TO_MAP_H_

#include <ros/ros.h>
#include <ar_track_alvar/AlvarMarkers.h>
#include <nav_msgs/OccupancyGrid.h>
#include <vector>

class markers_to_map
{
public:
  /*!
   * Creates a markers_to_map object which creates a map with obstacles with locations determined by ar markers. ROS nodes, services, and publishers
   * are created and maintained within this object.
   */
  markers_to_map();

private:

  //TODO: fix comments

  /*!
   * converter callback function.
   *
   * \param odom the message for the odom topic
   */
  void markers_cback(const ar_track_alvar::AlvarMarkers::ConstPtr& marker);

  ros::Subscriber markers_in; /*!< the markers topic */
  ros::Publisher map_out; /*!< the map topic */

};

//TODO: fix comments

/*!
 * Creates and runs the markers_to_map node.
 *
 * \param argc argument count that is passed to ros::init
 * \param argv arguments that are passed to ros::init
 * \return EXIT_SUCCESS if the node runs correctly
 */
int main(int argc, char **argv);

#endif //MARKERS_TO_MAP_H_

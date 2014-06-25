/*!
 * \markers_to_map.cpp
 * \brief places obstacles on a map at a location corresponding to an ar marker
 *
 * places obstacles on a map at a location corresponding to an ar marker
 *
 * \author Steven Kordell, WPI - spkordell@wpi.edu
 * \date June 25, 2014
 */


//TODO:Fix comments

#ifndef MARKERS_TO_MAP_H_
#define MARKERS_TO_MAP_H_

#include <vector>
#include <boost/lexical_cast.hpp>
#include <ros/ros.h>
#include <ar_track_alvar/AlvarMarkers.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#define PI 3.14159265358979323846  /* pi */

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

  ros::Subscriber markers_in; /*!< markers topic */
  ros::Subscriber map_in; /*!< map_in topic */
  ros::Publisher map_out; /*!< map_out topic */
  tf::TransformListener listener; /*!< transform listener */
  nav_msgs::OccupancyGrid globalMap; /*!< map used for determining parameters of output map */
  bool mapReceived; /*!< true when a map has been received */

  /*!
   * marker callback function: publishes a map with obstacles corresponding to ar markers
   * \param Markers markers registered by ar_track_alvar
   */
  void markers_cback(const ar_track_alvar::AlvarMarkers::ConstPtr& markers);

  /*!
   * callback for receiving the environment map, used for determining output map parameters
   * \param map The map
   */
  void map_in_cback(const nav_msgs::OccupancyGrid::ConstPtr& map);

  /*!
   * Rounds a floating point number to a specified precision, used for discretizing continuous values into grid cells
   * \param f The number to round
   * \param prec The precision to round the number to
   * \returns The input value rounded to the specified precision
   */
  float round(float f,float prec);

  /*!
   * Returns the minimum of two input values
   * \param a The first input value
   * \param b The second input value
   * \returns The smallest of the two inputs
   */
  float min(float a, float b);

  /*!
   * Returns the maximum of two input values
   * \param a The first input value
   * \param b The second input value
   * \returns The largest of the two inputs
   */
  float max(float a, float b);

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

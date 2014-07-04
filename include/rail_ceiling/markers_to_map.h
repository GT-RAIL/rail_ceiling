/*!
 * \markers_to_map.cpp
 * \brief places obstacles on a map at a location corresponding to an ar marker
 *
 * places obstacles on a map at a location corresponding to an ar marker
 *
 * \author Steven Kordell, WPI - spkordell@wpi.edu
 * \date June 25, 2014
 */

#ifndef MARKERS_TO_MAP_H_
#define MARKERS_TO_MAP_H_

#include <vector>
#include <tinyxml.h>
#include <boost/lexical_cast.hpp>
#include <ros/ros.h>
#include <ar_track_alvar/AlvarMarkers.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <rail_ceiling/bundle.h>
#include "opencv2/core/core.hpp"


#define PI 3.14159265358979323846  /* pi */


//TODO comment
struct layer_info_t {
  std::string name; /*! <The name of this layer */
  map_type_t mapType;
  ros::Publisher publisher;
  nav_msgs::OccupancyGrid* map;
  std::vector<signed char>* mapData;
};

class markers_to_map
{
public:
  /*!
   * Creates a markers_to_map object which creates a map with obstacles with locations determined by ar markers. ROS nodes, services, and publishers
   * are created and maintained within this object.
   */
  markers_to_map();

  /*!
   * Adds a bundle to the list of obstacle bundles
   *\param bundle The bundle to add
   */
  void addBundle(Bundle* bundle);

  /*!
   * Gets a bundle from the list of obstacle bundles
   *\param index Index of the bundle to get
   *\return A pointer to the bundle at the specified index
   */
  Bundle* getBundle(int index);

  /*!<
   * Returns the node's update rate
   *\returns The node's update rate
   */
  double getUpdateRate();

  //TODO: make private
  ros::Publisher footprint_out; /*< footprint polygon topic */

  //TODO:comment
  void createMapTopics();

private:
  ros::NodeHandle nh; /*!< a handle for this ros node */
  ros::Subscriber markers_in; /*!< markers topic */
  ros::Subscriber map_in; /*!< map_in topic */

  //TODO:comment
  std::vector<layer_info_t*> mapLayers;

  //TODO:remove
  //std::vector<ros::Publisher> out_maps; /*! < topics for each output map */


  tf::TransformListener listener; /*!< transform listener */
  nav_msgs::OccupancyGrid globalMap; /*!< map used for determining parameters of output obstacle map */
  bool globalMapReceived; /*!< true when a map has been received */
  double updateRate; /*!< Rate at which to update the obstacle map */
  std::vector<Bundle*> bundles; /*!< a list of all the obstacle bundles */

  //TODO: comment
  //std::vector<std::string> layerNames;

  /*!
   * marker callback function: publishes a map with obstacles corresponding to ar markers
   * \param markers Markers registered by ar_track_alvar
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
  float round(float f, float prec);

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

/*!
 * Creates and runs the markers_to_map node.
 *
 * \param argc argument count that is passed to ros::init
 * \param argv arguments that are passed to ros::init
 * \return EXIT_SUCCESS if the node runs correctly
 */
int main(int argc, char **argv);

#endif //MARKERS_TO_MAP_H_

/*!
 * \markers_to_map.h
 * \brief places obstacles on a map at a location corresponding to an ar marker
 *
 * places obstacles on a map at a location corresponding to an ar marker
 *
 * \author Steven Kordell, WPI - spkordell@wpi.edu
 * \date July 10, 2014
 */

#ifndef MARKERS_TO_MAP_H_
#define MARKERS_TO_MAP_H_

#include <vector>
#include <tinyxml.h>
#include <boost/lexical_cast.hpp>
#include <ros/ros.h>
#include <ar_track_alvar/AlvarMarkers.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <rail_ceiling/bundle.h>
#include <rail_ceiling/marker_callback_functor.h>
#include <rail_ceiling/marker_vis_callback_functor.h>
#include "opencv2/core/core.hpp"

#define PI 3.14159265358979323846  /* pi */

struct layer_info_t
{
  std::string name; /*! <The name of this layer */
  map_type_t mapType; /*! <The layer type */
  ros::Publisher publisher; /*! <Ros publisher for publishing the map */
  nav_msgs::OccupancyGrid* map; /*! <The map */
  std::vector<signed char>* mapData; /*<The map data */
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

  /*!
   * Returns the node's update rate
   *\returns The node's update rate
   */
  double getUpdateRate();

  /*!
   * Creates a global list of layers and generates the ros publishers for each layer
   */
  void initializeLayers();

  /*!
   * updates the maps with with obstacles corresponding to ar markers
   */
  void updateMarkerMaps();

private:
  ros::NodeHandle nh; /*!< a handle for this ros node */
  tf::TransformListener listener; /*!< transform listener */
  std::vector<ros::Subscriber> markers_in; /*!< list of input marker topics */
  std::vector<ros::Subscriber> vis_markers_in; /*!< list of input marker topics with camera information */
  ros::Subscriber map_in; /*!< map_in topic */
  nav_msgs::OccupancyGrid globalMap; /*!< the incoming static map of the area */
  bool globalMapReceived; /*!< true when a map has been received */
  std::vector<layer_info_t*> mapLayers; /*< A global list of all the map layers which will be published */
  std::vector<Bundle*> bundles; /*!< a list of all the obstacle bundles */
  std::vector<ar_track_alvar::AlvarMarkers::ConstPtr> markerDataIn; /*! < Incoming marker data from each camera. Poses are with respect to map. Contains only master markers. */
  std::vector<std::vector<visualization_msgs::Marker::ConstPtr> > markerVisDataIn; /*! < Incoming marker data from each camera. Poses are with respect to the camera. Contains all markers. */

  //parameters
  int cameraCount; /*!< the number of cameras */
  double updateRate; /*!< rate at which to update the obstacle map */
  double matchSizePublishPeriod; /*!< time (in seconds) between publications of layers of the match_size type */
  double matchDataPublishPeriod; /*!< time (in seconds) between publications of layers of the match_data type */
  double rollingPublishPeriod; /*!< time (in seconds) between publications of layers of the rolling type */
  double rollingMapWidth; /*! <width of the rolling map in meters */
  double rollingMapHeight; /*!< height of the rolling map in meters */
  std::string odomFrameId; /*! < robot's odometry frame (used for rolling map) */
  std::string baseFrameId; /*! < robot's base frame (used for rolling map) */

  //timers
  bool publishTimersStarted; /*!< state of the map publication timers */
  ros::Timer matchSizeTimer; /*!< timer for determining when to publish match size maps */
  ros::Timer matchDataTimer; /*!< timer for determining when to publish match data maps */
  ros::Timer rollingTimer; /*!< timer for determining when to publish rolling maps */

  /*!
   * Callback for the match size map publishing timer
   */
  void publishMatchSizeTimerCallback(const ros::TimerEvent&);

  /*!
   * Callback for the match data map publishing timer
   */
  void publishMatchDataTimerCallback(const ros::TimerEvent&);

  /*!
   * Callback for the rolling map publishing timer
   */
  void publishRollingTimerCallback(const ros::TimerEvent&);

  /*!
   * callback for receiving the static environment map
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

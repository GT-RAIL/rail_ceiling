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
#include <libgen.h>
#include <fstream>
#include <map_server/image_loader.h>
#include <yaml-cpp/yaml.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <rail_ceiling/bundle.h>
#include <rail_ceiling/marker_callback_functor.h>
#include <opencv2/core/core.hpp>
#include <move_base_msgs/MoveBaseAction.h>

#define PI 3.14159265358979323846  /* pi */

#ifdef HAVE_NEW_YAMLCPP
// The >> operator disappeared in yaml-cpp 0.5, so this function is
// added to provide support for code written under the yaml-cpp 0.3 API.
template<typename T>
void operator >> (const YAML::Node& node, T& i)
{
  i = node.as<T>();
}
#endif

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
  ros::Subscriber map_in; /*!< map_in topic */
  ros::Subscriber cmd_vel_in; /*!< the cmd_vel_in topic */
  ros::Subscriber nav_goal_in; /*!< the nav_goal_in topic */
  ros::Subscriber nav_goal_result; /*!< the nav_goal_out topic */
  ros::ServiceServer static_map_service; /*!< service for sending the static map */
  nav_msgs::OccupancyGrid globalMap; /*!< the incoming static map of the area */
  bool globalMapReceived; /*!< true when a map has been received */
  std::vector<layer_info_t*> mapLayers; /*< A global list of all the map layers which will be published */
  std::vector<Bundle*> bundles; /*!< a list of all the obstacle bundles */
  std::vector<ar_track_alvar_msgs::AlvarMarkers::ConstPtr> markerDataIn; /*! < Incoming marker data from each camera. Poses are with respect to map. Contains only master markers. */
  bool navigating; /*! < is the robot currently navigating */
  bool driving; /* < is the robot currently driving */

  //parameters
  int cameraCount; /*!< the number of cameras */
  double updateRate; /*!< rate at which to update the obstacle map */
  double matchSizePublishPeriod; /*!< time (in seconds) between publications of layers of the match_size type */
  double matchDataPublishPeriod; /*!< time (in seconds) between publications of layers of the match_data type */
  double rollingPublishPeriod; /*!< time (in seconds) between publications of layers of the rolling type */
  double rollingMapWidth; /*! <width of the rolling map in meters */
  double rollingMapHeight; /*!< height of the rolling map in meters */
  bool loadStaticMapFromFile; /*! < if true, node will load the map from a file rather than getting it from a topic */
  std::string staticMapYamlFile; /*! < path to the map file to load if loading the map from a file */
  bool getMapWithService; /*! <if true, node will respond to service calls to "static_map" by sending the map layer specified by the "layer_to_send_on_service_call" parameter */
  std::string layerToSendOnServiceCall; /*< name of the layer to send on service calls to "static_map" */
  bool dontPublishWhileNavigating; /*!< Setting to true will prevent the node from publishing new maps of the match_data layer type while the robot is navigating, possibly preventing localization issues */
  bool dontPublishWhileDriving; /*!< Setting to true will prevent the node from publishing new maps of the match_data layer type while the robot is driving, possibly preventing localization issues */
  double drivingTimeout; /*! <Time after receiving last command velocity to allow the publication of match_data maps again */
  std::string odomFrameId; /*! < robot's odometry frame (used for rolling map) */
  std::string baseFrameId; /*! < robot's base frame (used for rolling map) */

  //timers
  bool publishTimersStarted; /*!< state of the map publication timers */
  ros::Timer matchSizeTimer; /*!< timer for determining when to publish match size maps */
  ros::Timer matchDataTimer; /*!< timer for determining when to publish match data maps */
  ros::Timer rollingTimer; /*!< timer for determining when to publish rolling maps */
  ros::Timer cmdVelTimer; /*!< timer for determining when to allow publishing of match data maps after recieving comand velocities */

  /*
   * Combines the markers from multiple cameras into a single list of markers
   *\returns The list of markers from all the cameras
   */
  ar_track_alvar_msgs::AlvarMarkers* mergeMarkerData();

  /*
   * Initializes the various maps
   */
  void initializeMaps();

  /*
   * Loads a map from a yaml file
   */
  nav_msgs::OccupancyGrid loadMapFromFile(const std::string& fname);

  /*
   * Sends the map specified by the layer_to_send_on_service_call parameter
   */
  bool staticMapServiceCallback(nav_msgs::GetMap::Request &req, nav_msgs::GetMap::Response &res);

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
   * Callback for cmd vel timer
   */
  void cmdVelTimerCallback(const ros::TimerEvent&);

  /*!
   * callback for receiving the static environment map
   * \param map The map
   */
  void map_in_cback(const nav_msgs::OccupancyGrid::ConstPtr& map);

  /*!
   * callback for receiving command velocities
   * \param vel The cmd_vel
   */
  void cmd_vel_cback(const geometry_msgs::Twist::ConstPtr& vel);

  /*!
   * navigation goal callback function
   * \param nav_goal The navigation goal received
   */
  void nav_goal_cback(const geometry_msgs::PoseStamped::ConstPtr& nav_goal);

  /*!
   * navigation result callback
   * \param The navigation result
   */
  void nav_goal_result_cback(const move_base_msgs::MoveBaseActionResult::ConstPtr& result);

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

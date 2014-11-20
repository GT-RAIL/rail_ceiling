/*!
 * \file furniture_tracker.h
 * \brief AR-tagged furniture tracking from overhead cameras.
 *
 * rail_lab_location_server creates an action server with pre-defined goal locations for CARL navigation.
 *
 * \author David Kent, WPI - davidkent@wpi.edu
 * \date October 30, 2014
 */

#ifndef FURNITURE_TRACKER_H_
#define FURNITURE_TRACKER_H_

#include <ros/ros.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/Pose2D.h>
#include <rail_ceiling/GetAllObstacles.h>
#include <rail_ceiling/Obstacles.h>
#include <ros/package.h>
#include <tf/transform_listener.h>
#include <yaml-cpp/yaml.h>

#define POSITION_THRESHOLD .03 //threshold at which to republish changes in furniture position
#define ANGULAR_THRESHOLD .087 //threshold at which to republish changes in furniture angle, ~5 degrees

typedef struct _Marker
{
  unsigned int id;
  unsigned int fid; //associated furniture id
  geometry_msgs::Pose2D pose;
} Marker;

typedef struct _Furniture
{
  unsigned int id;
  std::string type;
  std::vector<Marker> markers;
} Furniture;

typedef struct _FurnitureTransforms
{
  std::string name;
  std::vector<geometry_msgs::Polygon> localizationFootprint;
  std::vector<geometry_msgs::Polygon> navigationFootprint;
} FurnitureTransforms;

class FurnitureTracker
{
public:
  /**
  * \brief Constructor, initializes data structures containing information about markers and furniture obstacles
  */
  FurnitureTracker();

  /**
  * \brief publish localization and navigation data for furniture that has recently changed pose
  *
  * Furniture poses will be published initially as markers are detected, and subsequently will only be republished
  * if the position exceeds the POSITION_THRESHOLD or if the angle exceeds the ANGULAR_THRESHOLD.
  */
  void publishFurniturePoses();

private:
  ros::NodeHandle n;

  ros::Publisher furnitureLayerPub;
  ros::ServiceServer allPosesServer;
  std::vector<ros::Subscriber> markerSubscribers;

  tf::TransformListener tfListener;

  bool readInitialPoses;
  std::vector<Marker> markerList;
  std::vector<Furniture> furnitureList;
  std::vector<FurnitureTransforms> footprintTransforms;
  std::vector<geometry_msgs::Pose2D> furniturePoses;
  std::vector<geometry_msgs::Pose2D> lastPublishedPoses;

  /**
  * \brief read marker and furniture configuration data
  *
  * @param markerConfigFile configuration file path for markers on furniture
  * @param furnitureConfigFile configuration file for localization and navigation furniture footprints
  */
  void readConfigFiles(std::string markerConfigFile, std::string furnitureConfigFile);

  /**
  * \brief save updated marker data
  *
  * @param msg marker pose data
  */
  void markerCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg);

  /**
  * \brief service callback for returning the most recent set of previously published furniture poses
  *
  * This service allows nodes that subscribe late to the topic to get the furniture poses that were updated before
  * subscribing to the localization layer and navigation layer publishers.
  *
  * @param req service request denoting whether the request is for localization or navigation obstacle information
  * @param res service response including footprint information
  */
  bool getAllPoses(rail_ceiling::GetAllObstacles::Request &req, rail_ceiling::GetAllObstacles::Response &res);

  /**
  * \brief update obstacle message data
  *
  * @param index furniture id
  * @param type furniture type
  * @param obstacles message for obstacle information to be used for localization and navigation
  */
  void updateMessages(int index, std::string type, rail_ceiling::Obstacles *obstacles);

  /**
  * \brief Calculate footprint polygons and fill them into an obstacles message
  *
  * @param index furniture id
  * @param footprints furniture footprint information
  * @param obstacles obstacles message in which to store the calculated footprints
  * @param isLocalization true if this update is for localization footprints, false if it is for navigation footprints
  */
  void fillFootprintInformation(int index, std::vector<geometry_msgs::Polygon> footprints, rail_ceiling::Obstacles *obstacles, bool isLocalization);

  /**
  * \brief determine whether a pose has been updated after its initialization
  *
  * This function assumes that the pose will never be exactly (0.0, 0.0, 0.0) after it's been updated, which for the
  * purposes of furniture tracking is likely to be accurate.
  *
  * @param pose the pose to check
  * @return true if the pose has been previously updated
  */
  bool updated(geometry_msgs::Pose2D pose);
};

/*!
 * Creates and runs the furniture_tracker node.
 *
 * \param argc argument count that is passed to ros::init
 * \param argv arguments that are passed to ros::init
 * \return EXIT_SUCCESS if the node runs correctly
 */
int main(int argc, char **argv);

#endif

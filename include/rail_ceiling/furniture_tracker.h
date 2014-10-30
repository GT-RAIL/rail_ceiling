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
#include <geometry_msgs/Pose2D.h>
#include <ros/package.h>
#include <yaml-cpp/yaml.h>

typedef struct _Marker
{
  int id;
  geometry_msgs::Pose2D pose;
} Marker;

typedef struct _Furniture
{
  int id;
  std::string type;
  std::vector<Marker> markers;
} Furniture;

class FurnitureTracker
{
public:
  FurnitureTracker();

private:
  std::vector<Furniture> furnitureList;
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

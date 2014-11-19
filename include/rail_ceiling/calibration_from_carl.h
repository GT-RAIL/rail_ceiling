/*!
 * \file calibration_from_carl.h
 * \brief External camera calibration from an AR tag with known position on the CARL robot
 *
 * \author David Kent, WPI - davidkent@wpi.edu
 * \date November 14, 2014
 */

#ifndef CALIBRATION_FROM_CARL_H_
#define CALIBRATION_FROM_CARL_H_

#include <ros/ros.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <std_msgs/Int16.h>
#include <fstream>

/*!
 * \def FIXED_LINK_NAME
 * The link name of the cameras.
 */
#define CAMERA_LINK_NAME "calibration_ceiling_camera_"

#define REQUIRED_SAMPLES 25

class CalibrationFromCarl
{
public:
  /**
  * \brief Constructor
  */
  CalibrationFromCarl();

  void publishTransforms();

private:
  ros::NodeHandle n;

  ros::Subscriber startCalibrationSubscriber;
  std::vector<ros::Subscriber> markerSubscribers;

  tf::TransformListener tfListener;
  tf::TransformBroadcaster br; /*!< main transform broadcaster */

  int markerID;
  bool calibrationComplete;
  std::vector< std::vector<tf::StampedTransform> > transformSamples;
  std::vector<tf::StampedTransform> finalTransforms;
  std::vector<bool> calibrated;
  std::vector<bool> calibrationEnabled;

  /**
  * \brief enable calibration of a camera specified by the number in the message
  *
  * @param msg message containing camera number to begin calibrating
  */
  void startCalibrationCallback(const std_msgs::Int16::ConstPtr& msg);

  /**
  * \brief get samples if calibration is active
  *
  * @param msg marker pose data
  */
  void markerCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg);
};

/*!
 * Creates and runs the calibration_from_carl node.
 *
 * \param argc argument count that is passed to ros::init
 * \param argv arguments that are passed to ros::init
 * \return EXIT_SUCCESS if the node runs correctly
 */
int main(int argc, char **argv);

#endif

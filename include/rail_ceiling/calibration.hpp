/*!
 * \file calibration.hpp
 * \brief Allows for calibration of the ceiling cameras using AR markers.
 *
 * The calibration node uses AR tags in a fixed world location to determine the location of the cameras. These locations
 * are then printed to the terminal for use in a URDF. It is assumed the marker tags are published in the same frame as
 * the camera.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date August 22, 2014
 */

#ifndef CALIBRATION_HPP_
#define CALIBRATION_HPP_

#include <ros/ros.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <geometry_msgs/Pose.h>
#include <vector>
#include <string>
#include <tf/transform_broadcaster.h>

/*!
 * \def URDF
 * The URDF file to write.
 */
#define URDF "ceiling.urdf.xacro"

/*!
 * \def FIXED_LINK_NAME
 * The link name of the AR markers.
 */
#define FIXED_LINK_NAME "fixed_calibration_marker_"

/*!
 * \def FIXED_LINK_NAME
 * The link name of the cameras.
 */
#define CAMERA_LINK_NAME "calibration_ceiling_camera_"

/*!
 * \class calibration
 * \brief The main calibration object.
 *
 * The calibration object subscribes to all necessary topics to calibrate the ceiling cameras.
 */
class calibration
{
public:
  /*!
   * \brief Creates a calibration object.
   *
   * Creates a calibration object that can be used to calibrate a number of ceiling cameras. ROS nodes, services, and
   * publishers are created and maintained within this object.
   */
  calibration();

  /*!
   * \brief Publish known fixed frames of the markers to a global frame and the averages from the AR tracker.
   *
   * Publish known fixed frames of the markers to a global frame and the averages from the AR tracker. This will allow
   * later calculations to these frames.
   */
  void publish_tf();

  /*!
   * \brief Attempt to calibrate the cameras.
   *
   * Attempt to calibrate the cameras. Cameras will calibrate once enough samples have come in.
   */
  void attempt_calibration();

private:
  /*!
   * \brief AR marker topic callback function.
   *
   * Process and store the incoming AR marker positions from each camera.
   *
   * \param msg the message for the marker topic
   * \param camera the camera number
   */
  void marker_cback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg, int camera);

  /*!
   * \brief Write the calibration settings to a URDF file.
   *
   * Writes a file called 'ceiling.urdf.xacro' with the calibration settings.
   */
  void write_calibration();

  bool calibrated_; /*!< a flag to check if the calibration is complete */

  tf::TransformBroadcaster br; /*!< main transform broadcaster */

  ros::NodeHandle nh_, pnh_; /*!< a handle for this ROS node and the private node handle */

  int num_cameras_, num_samples_; /*!< the number of ceiling cameras and samples to take */
  std::string fixed_frame_, camera_frame_id_prefix_; /*!< the fixed frame of the markers and frame of the cameras */

  std::vector<ros::Subscriber> marker_subs_; /*!< the subscriptions to the marker topic */
  std::vector<geometry_msgs::Pose> fixed_poses_, average_poses_; /*!< the known fixed pose of each marker and average from the AR tracker */
  std::vector<std::vector<geometry_msgs::Pose> > samples_; /*!< the positions saved from the cameras */
};

/*!
 * Creates and runs the calibration node.
 *
 * \param argc argument count that is passed to ros::init
 * \param argv arguments that are passed to ros::init
 * \return EXIT_SUCCESS if the node runs correctly
 */
int main(int argc, char **argv);

#endif

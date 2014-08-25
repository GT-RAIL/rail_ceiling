/*!
 * \file calibration.cpp
 * \brief Allows for calibration of the ceiling cameras using AR markers.
 *
 * The calibration node uses AR tags in a fixed world location to determine the location of the cameras. These locations
 * are then printed to the terminal for use in a URDF. It is assumed the marker tags are published in the same frame as
 * the camera.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date August 22, 2014
 */

#include <rail_ceiling/calibration.hpp>
#include <tf/transform_listener.h>
#include <sstream>
#include <fstream>

using namespace std;

calibration::calibration() :
    pnh_("~")
{
  this->calibrated_ = false;

  // grab the number of cameras and samples to take
  pnh_.param("fixed_frame", this->fixed_frame_, string("map"));
  pnh_.param("camera_frame_id_prefix", this->camera_frame_id_prefix_, string("ceiling_cam_"));
  pnh_.param("num_cameras", this->num_cameras_, 1);
  pnh_.param("num_samples", this->num_samples_, 10);

  // subscribe to each marker topic and grab the parameters
  for (int i = 0; i < this->num_cameras_; i++)
  {
    // construct the topic and parameter names
    stringstream topic_ss, x_pos_ss, y_pos_ss, z_pos_ss, x_rot_ss, y_rot_ss, z_rot_ss, w_rot_ss;
    topic_ss << "ceiling_cam_tracker_" << i << "/ar_pose_marker";
    x_pos_ss << "ceiling_cam_" << i << "_pos_x";
    y_pos_ss << "ceiling_cam_" << i << "_pos_y";
    z_pos_ss << "ceiling_cam_" << i << "_pos_z";
    x_rot_ss << "ceiling_cam_" << i << "_rot_x";
    y_rot_ss << "ceiling_cam_" << i << "_rot_y";
    z_rot_ss << "ceiling_cam_" << i << "_rot_z";
    w_rot_ss << "ceiling_cam_" << i << "_rot_w";

    // create the subscription
    ros::Subscriber sub = this->nh_.subscribe<ar_track_alvar_msgs::AlvarMarkers>(
        topic_ss.str(), 1, boost::bind(&calibration::marker_cback, this, _1, i));
    // add it to the list
    this->marker_subs_.push_back(sub);
    // add a samples vector
    vector<geometry_msgs::Pose> samples;
    this->samples_.push_back(samples);

    // search for parameters
    geometry_msgs::Pose pose;
    pnh_.param(x_pos_ss.str(), pose.position.x, 0.0);
    pnh_.param(y_pos_ss.str(), pose.position.y, 0.0);
    pnh_.param(z_pos_ss.str(), pose.position.z, 0.0);
    pnh_.param(x_rot_ss.str(), pose.orientation.x, 0.0);
    pnh_.param(y_rot_ss.str(), pose.orientation.y, 0.0);
    pnh_.param(z_rot_ss.str(), pose.orientation.z, 0.0);
    pnh_.param(w_rot_ss.str(), pose.orientation.w, 1.0);
    this->fixed_poses_.push_back(pose);
  }

  ROS_INFO("Waiting to find %i samples of the calibration markers for each camera...", this->num_samples_);
}

void calibration::marker_cback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg, int camera)
{
  // check if we need anymore samples
  vector<geometry_msgs::Pose> &samples = this->samples_.at(camera);
  if (samples.size() < this->num_samples_)
  {
    // search for the correct marker for this camera
    int dest_marker = camera + 100;
    for (int i = 0; i < msg->markers.size(); i++)
    {
      ar_track_alvar_msgs::AlvarMarker marker = msg->markers.at(i);
      if (marker.id == dest_marker)
        // save the pose
        samples.push_back(marker.pose.pose);
    }
  }
}

void calibration::publish_tf()
{
  // go through each marker
  for (int i = 0; i < this->num_cameras_; i++)
  {
    // publish the fixed pose
    geometry_msgs::Pose &fixed_pose = this->fixed_poses_.at(i);
    // create a TF
    tf::Transform tf_fixed;
    tf_fixed.setOrigin(tf::Vector3(fixed_pose.position.x, fixed_pose.position.y, fixed_pose.position.z));
    tf_fixed.setRotation(
        tf::Quaternion(fixed_pose.orientation.x, fixed_pose.orientation.y, fixed_pose.orientation.z,
                       fixed_pose.orientation.w));
    // publish the fixed pose TF
    stringstream ss_fixed;
    ss_fixed << FIXED_LINK_NAME << i;
    br.sendTransform(tf::StampedTransform(tf_fixed, ros::Time::now(), this->fixed_frame_, ss_fixed.str()));

    // publish the average pose from the camera if ready
    if (this->average_poses_.size() > i)
    {
      // publish the average pose
      geometry_msgs::Pose &average_pose = this->average_poses_.at(i);
      // create a TF
      tf::Transform tf_average;
      tf_average.setRotation(tf::Quaternion(0, 0, 0, 1));
      tf_average.setOrigin(tf::Vector3(average_pose.position.x, average_pose.position.y, average_pose.position.z));
      tf_average.setRotation(
          tf::Quaternion(average_pose.orientation.x, average_pose.orientation.y, average_pose.orientation.z,
                         average_pose.orientation.w).normalize());
      // now invert it
      tf::Transform tf_average_inverse = tf_average.inverse();
      stringstream ss_camera;
      ss_camera << CAMERA_LINK_NAME << i;
      br.sendTransform(tf::StampedTransform(tf_average_inverse, ros::Time::now(), ss_fixed.str(), ss_camera.str()));
    }
  }
}

void calibration::attempt_calibration()
{
  // check if we finished
  if (!this->calibrated_)
  {
    // check for all the samples
    bool ready = true;
    for (int i = 0; i < this->num_cameras_; i++)
      ready &= this->samples_.at(i).size() >= this->num_samples_;

    if (ready)
    {
      ROS_INFO("Sample collection complete.");

      // unsubscribe
      for (int i = 0; i < this->num_cameras_; i++)
        this->marker_subs_.at(i).shutdown();

      // calculate the average pose
      for (int i = 0; i < this->num_cameras_; i++)
      {
        geometry_msgs::Pose pose;
        vector<geometry_msgs::Pose> &samples = this->samples_.at(i);
        for (int j = 0; j < this->num_samples_; j++)
        {
          // calculate the average as we go
          geometry_msgs::Pose &sample = samples.at(j);
          int n = j + 1;
          pose.position.x = (((n - 1) * pose.position.x + sample.position.x) / (float)n);
          pose.position.y = (((n - 1) * pose.position.y + sample.position.y) / (float)n);
          pose.position.z = (((n - 1) * pose.position.z + sample.position.z) / (float)n);
          pose.orientation.w = (((n - 1) * pose.orientation.w + sample.orientation.w) / (float)n);
          pose.orientation.x = (((n - 1) * pose.orientation.x + sample.orientation.x) / (float)n);
          pose.orientation.y = (((n - 1) * pose.orientation.y + sample.orientation.y) / (float)n);
          pose.orientation.z = (((n - 1) * pose.orientation.z + sample.orientation.z) / (float)n);
        }
        // save the pose
        this->average_poses_.push_back(pose);
      }

      // publish transforms from the marker to the camera
      this->publish_tf();

      // write the calibration file
      this->write_calibration();

      this->calibrated_ = true;
      ROS_INFO("Calibration complete!");
    }
  }
}

void calibration::write_calibration()
{
  // get the current TFs
  tf::TransformListener listener;
  vector<tf::StampedTransform> tfs;

  for (int i = 0; i < this->num_cameras_; i++)
  {
    stringstream ss_frame;
    ss_frame << CAMERA_LINK_NAME << i;

    // wait for the TF to come back
    bool found = false;
    while (!found)
    {
      try
      {
        // try and get the frame
        tf::StampedTransform tf;
        listener.lookupTransform(this->fixed_frame_, ss_frame.str(), ros::Time(0), tf);
        tfs.push_back(tf);
        found = true;
      }
      catch (tf::TransformException &ex)
      {
        // republish the TF
        this->publish_tf();
        // sleep and continue
        ros::Duration(1.0).sleep();
      }
    }
  }

  stringstream ss;
  ss << getenv("HOME") << "/" << URDF;
  string file_name = ss.str();

  // open the file for writing
  ofstream urdf;
  urdf.open(file_name.c_str());
  if (!urdf.is_open())
    ROS_ERROR("Failed to open '~/%s' for writing.", file_name.c_str());
  else
  {
    urdf << "<?xml version=\"1.0\"?>\n";
    urdf << "<robot xmlns:xacro=\"http://www.ros.org/wiki/xacro\" name=\"ceiling\">\n\n";

    urdf << "  <!-- Auto-Generated from rail_ceiling/calibration Node -->\n\n";

    urdf << "  <xacro:include filename=\"$(find rail_ceiling)/urdf/camera.urdf.xacro\" />\n\n";

    urdf << "  <xacro:property name=\"PARENT\" value=\"" << this->fixed_frame_ << "\" />\n\n";

    urdf << "  <!-- fixed frame -->\n";
    urdf << "  <link name=\"${PARENT}\" />\n\n";

    urdf << "  <!-- " << this->num_cameras_ << " Camera(s) -->\n";
    for (int i = 0; i < this->num_cameras_; i++)
    {
      // grab the TF info
      tf::StampedTransform &tf = tfs.at(i);
      tf::Vector3 &pos = tf.getOrigin();
      double roll, pitch, yaw;
      tf.getBasis().getRPY(roll, pitch, yaw);
      urdf << "  <xacro:ceiling_cam parent=\"${PARENT}\" link=\"" << this->camera_frame_id_prefix_ << i << "\">\n";
      urdf << "    <origin xyz=\"" << pos.getX() << " " << pos.getY() << " " << pos.getZ() << "\" rpy=\"" << roll << " "
          << pitch << " " << yaw << "\" />\n";
      urdf << "  </xacro:ceiling_cam>\n";
    }
    urdf << "</robot>\n\n";

    urdf.close();
    ROS_INFO("Calibration written to '%s'.", file_name.c_str());
  }
}

int main(int argc, char **argv)
{
  // initialize ROS and the node
  ros::init(argc, argv, "calibration");

  // initialize the calibration object
  calibration calib;

  // continue until a ctrl-c has occurred
  ros::Rate r(60);
  while (ros::ok())
  {
    calib.publish_tf();
    calib.attempt_calibration();
    ros::spinOnce();
    r.sleep();
  }

  return EXIT_SUCCESS;
}

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
#include <sstream>

using namespace std;

calibration::calibration() :
    pnh_("~")
{
  // grab the number of cameras and samples to take
  pnh_.param("num_cameras", this->num_cameras_, 1);
  pnh_.param("num_samples", this->num_samples_, 25);

  // subscribe to each marker topic
  for (int i = 0; i < this->num_cameras_; i++)
  {
    // construct the topic name
    stringstream ss;
    ss << "ceiling_cam_tracker_" << i << "/ar_pose_marker";
    string topic = ss.str();

    // create the subscription
    ros::Subscriber sub = this->nh_.subscribe<ar_track_alvar_msgs::AlvarMarkers>(
        topic, 1, boost::bind(&calibration::marker_cback, this, _1, i));
    // add it to the list
    this->marker_subs_.push_back(sub);
    // add a samples vector
    vector<geometry_msgs::Pose> samples;
    this->samples_.push_back(samples);
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

void calibration::attempt_calibration()
{
  // check for all the samples
  bool ready = true;
  for (int i = 0; i < this->num_cameras_; i++)
    ready &= this->samples_.at(i).size() >= this->num_samples_;

  if (ready)
  {
    ROS_INFO("Sample collection complete!");

    // unsubscribe
    for (int i = 0; i < this->num_cameras_; i++)
      this->marker_subs_.at(i).shutdown();

    // calculate the average pose
    geometry_msgs::Pose poses[this->num_cameras_];
    for (int i = 0; i < this->num_cameras_; i++)
    {
      geometry_msgs::Pose &pose = poses[i];
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
    }

    // we are finished
    this->nh_.shutdown();
    ros::shutdown();
  }
}

int main(int argc, char **argv)
{
// initialize ROS and the node
  ros::init(argc, argv, "calibration");

// initialize the calibration object
  calibration calib;

// continue until a ctrl-c has occurred
  ros::Rate r(120);
  while (ros::ok())
  {
    calib.attempt_calibration();
    ros::spinOnce();
    r.sleep();
  }

  return EXIT_SUCCESS;
}

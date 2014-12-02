/*!
 * \file calibration_from_carl.cpp
 * \brief External camera calibration from an AR tag with known position on the CARL robot
 *
 * \author David Kent, WPI - davidkent@wpi.edu
 * \date November 14, 2014
 */
#include <rail_ceiling/calibration_from_carl.h>

using namespace std;

CalibrationFromCarl::CalibrationFromCarl()
{
  // private node handle
  ros::NodeHandle private_nh("~");

  // get number of marker topics (i.e. number of overhead cameras), and the id of the calibration marker
  int numCameras;
  private_nh.param("num_cameras", numCameras, 5);
  private_nh.param("calibration_marker_id", markerID, 200);

  calibrationEnabled.resize(numCameras);
  transformSamples.resize(numCameras);
  finalTransforms.resize(numCameras);
  calibrated.resize(numCameras);
  markerSubscribers.resize(numCameras);
  for (unsigned int i = 0; i < numCameras; i ++)
  {
    transformSamples[i].clear();
    calibrationEnabled[i] = false;
    calibrated[i] = false;
    stringstream topicStream;
    topicStream << "ceiling_cam_tracker_" << i << "/ar_pose_marker";
    markerSubscribers[i] = n.subscribe(topicStream.str(), 1, &CalibrationFromCarl::markerCallback, this);
  }

  startCalibrationSubscriber = n.subscribe("start_calibration", 1, &CalibrationFromCarl::startCalibrationCallback, this);
  calibrationComplete = false;
}

void CalibrationFromCarl::startCalibrationCallback(const std_msgs::Int16::ConstPtr& msg)
{
  //enable calibration for the specified camera and clear out any previous samples
  calibrationEnabled[msg->data] = true;
  calibrated[msg->data] = false;
  transformSamples[msg->data].clear();
  calibrationComplete = false;
}

void CalibrationFromCarl::markerCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg)
{
  for (unsigned int i = 0; i < msg->markers.size(); i ++)
  {
    //check if the marker detected is the calibration marker
    if (msg->markers[i].id == markerID)
    {
      //get the marker index, this assumes the camera coordinate frames contain the string "cam_n", where n is an
      //integer denoting the camera number
      size_t pos = msg->markers[i].header.frame_id.find("cam_");
      int cameraid = atoi((msg->markers[i].header.frame_id.substr(pos + 4)).c_str());
      if (calibrationEnabled[cameraid])
      {
        geometry_msgs::PoseStamped sample = msg->markers[i].pose;
        sample.header.frame_id=msg->markers[i].header.frame_id;

        // transform pose to map frame
        tf::Transform tfSample;
        tf::StampedTransform finalTransform;
        tfSample.setOrigin(tf::Vector3(sample.pose.position.x, sample.pose.position.y, sample.pose.position.z));
        tfSample.setRotation(tf::Quaternion(sample.pose.orientation.x, sample.pose.orientation.y, sample.pose.orientation.z, sample.pose.orientation.w).normalize());
        // invert it
        tf::Transform tfSampleInverse = tfSample.inverse();
        ros::Time time = ros::Time::now();
        br.sendTransform(tf::StampedTransform(tfSampleInverse, time, "calibration_link", sample.header.frame_id.c_str()));
        tfListener.waitForTransform("map", "calibration_link", time, ros::Duration(1.0));
        tfListener.lookupTransform("map", sample.header.frame_id, time, finalTransform);

        transformSamples[cameraid].push_back(finalTransform);
        if (transformSamples[cameraid].size() >= REQUIRED_SAMPLES)
        {
          ROS_INFO("Finished calibration for camera %d", cameraid);
          calibrationEnabled[cameraid] = false;
        }
      }
    }
  }
}

void CalibrationFromCarl::publishTransforms()
{
  // go through each marker
  bool finished = true;
  for (unsigned int i = 0; i < transformSamples.size(); i ++)
  {
    // publish the average pose from the camera if it's received enough samples
    if (transformSamples[i].size() >= REQUIRED_SAMPLES)
    {
      if (!calibrated[i])
      {
        //calculate average pose
        //TODO: find a better way to approximate average rotation in 3D
        tf::StampedTransform avgTransform;
        avgTransform.frame_id_ = transformSamples[i][0].frame_id_;
        avgTransform.child_frame_id_ = transformSamples[i][0].child_frame_id_;
        avgTransform.stamp_ = ros::Time::now();
        float x = 0.0, y = 0.0, z = 0.0;
        //float qx = 0.0, qy = 0.0, qz = 0.0, qw = 0.0;
        tf::Quaternion avgQuat;
        for (unsigned int j = 0; j < transformSamples[i].size(); j++)
        {
          x += transformSamples[i][j].getOrigin().x();
          y += transformSamples[i][j].getOrigin().y();
          z += transformSamples[i][j].getOrigin().z();
          if (j == 0)
          {
            avgQuat = transformSamples[i][j].getRotation();
          }
          else
          {
            avgQuat.slerp(transformSamples[i][j].getRotation(), 1.0/((float)(j + 1)));
          }
          /*
          qx += transformSamples[i][j].getRotation().getX();
          qy += transformSamples[i][j].getRotation().getY();
          qz += transformSamples[i][j].getRotation().getZ();
          qw += transformSamples[i][j].getRotation().getW();
          */
        }

        int numSamples = transformSamples[i].size();
        avgTransform.setOrigin(tf::Vector3(x/numSamples, y/numSamples, z/numSamples));
        //avgTransform.setRotation(tf::Quaternion(qx/numSamples, qy/numSamples, qz/numSamples, qw/numSamples).normalize());
        avgTransform.setRotation(avgQuat);

        finalTransforms[i] = avgTransform;
        calibrated[i] = true;
      }

      br.sendTransform(finalTransforms[i]);
    }
    else
    {
      finished = false;
    }
  }

  if (finished && !calibrationComplete)
  {
    //write calibration file
    ROS_INFO("Writing calibration...");

    stringstream ss;
    //TODO: update
    ss << getenv("HOME") << "/testCalibrationFile.urdf";
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

      urdf << "  <xacro:property name=\"PARENT\" value=\"" << "map" << "\" />\n\n";

      urdf << "  <!-- fixed frame -->\n";
      urdf << "  <link name=\"${PARENT}\" />\n\n";

      urdf << "  <!-- " << finalTransforms.size() << " Camera(s) -->\n";
      for (int i = 0; i < finalTransforms.size(); i++)
      {
        // grab the TF info
        tf::StampedTransform &tf = finalTransforms.at(i);
        tf::Vector3 &pos = tf.getOrigin();
        double roll, pitch, yaw;
        tf.getBasis().getRPY(roll, pitch, yaw);
        urdf << "  <xacro:ceiling_cam parent=\"${PARENT}\" link=\"" << tf.child_frame_id_ << i << "\">\n";
        urdf << "    <origin xyz=\"" << pos.getX() << " " << pos.getY() << " " << pos.getZ() << "\" rpy=\"" << roll << " "
            << pitch << " " << yaw << "\" />\n";
        urdf << "  </xacro:ceiling_cam>\n";
      }
      urdf << "</robot>\n\n";

      urdf.close();
      ROS_INFO("Calibration written to '%s'.", file_name.c_str());
    }

    calibrationComplete = true;
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "calibration_from_carl");

  CalibrationFromCarl c;

  ros::Rate loop_rate(30);
  while (ros::ok())
  {
    c.publishTransforms();
    ros::spinOnce();
    loop_rate.sleep();
  }
}

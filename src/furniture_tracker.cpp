/*!
 * \file furniture_tracker.cpp
 * \brief AR-tagged furniture tracking from overhead cameras.
 *
 * rail_lab_location_server creates an action server with pre-defined goal locations for CARL navigation.
 *
 * \author David Kent, WPI - davidkent@wpi.edu
 * \date October 30, 2014
 */
#include <rail_ceiling/furniture_tracker.h>

using namespace std;

FurnitureTracker::FurnitureTracker()
{
  // private node handle
  ros::NodeHandle private_nh("~");

  // grab the config file
  stringstream ss;
  ss << ros::package::getPath("rail_ceiling") << "/config/markers.yaml";
  string file;
  private_nh.param("markers_config", file, ss.str());

  // parse the configuration file

//#ifdef YAMLCPP_GT_0_5_0
  YAML::Node config = YAML::LoadFile(file);
  for (size_t i = 0; i < config.size(); i++)
  {
    Furniture f;

    // load the ID and type
    f.id = config[i]["id"].as<int>();
    f.type = config[i]["type"].as<std::string>();

    // load marker information
    YAML::Node markers = config[i]["markers"];
    f.markers.resize(markers.size());
    for (size_t j = 0; j < markers.size(); j ++)
    {
      f.markers[j].id = markers[j]["id"].as<int>();
      f.markers[j].pose.x = markers[j]["x"].as<double>();
      f.markers[j].pose.y = markers[j]["y"].as<double>();
      f.markers[j].pose.theta = markers[j]["theta"].as<double>();
    }

    // store the furniture piece
    furnitureList.push_back(f);
  }
/*
#else
  ifstream fin(file.c_str());
  YAML::Parser parser(fin);
  YAML::Node config;
  parser.GetNextDocument(config);
  for (size_t i = 0; i < config.size(); i++)
  {
    int id;
    config[i]["position"] >> id;
    string name;
    config[i]["position"] >> name;

    geometry_msgs::Pose pose;
    const YAML::Node &position = config[i]["position"];
    position[0] >> pose.position.x;
    position[1] >> pose.position.y;
    position[2] >> pose.position.z;
    const YAML::Node &orientation = config[i]["orientation"];
    orientation[0] >> pose.orientation.x;
    orientation[1] >> pose.orientation.y;
    orientation[2] >> pose.orientation.z;
    orientation[3] >> pose.orientation.w;

    locations_.push_back(location(id, name, pose));
  }
#endif
*/

  for (unsigned int i = 0; i < furnitureList.size(); i ++)
  {
    ROS_INFO("Furniture piece id: %d", furnitureList[i].id);
    ROS_INFO("\tType: %s", furnitureList[i].type.c_str());
    ROS_INFO("\tMarkers:");
    for (unsigned int j = 0; j < furnitureList[i].markers.size(); j ++)
    {
      ROS_INFO("\t\t%d : (%f, %f, %f)", furnitureList[i].markers[j].id, furnitureList[i].markers[j].pose.x, furnitureList[i].markers[j].pose.y, furnitureList[i].markers[j].pose.theta);
    }
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "furniture_tracker");

  FurnitureTracker ft;

  ros::spin();
}

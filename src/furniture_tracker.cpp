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
#include <fstream>

using namespace std;

FurnitureTracker::FurnitureTracker()
{
  // private node handle
  ros::NodeHandle private_nh("~");

  // get number of marker topics (i.e. number of overhead cameras)
  int numMarkerTopics;
  private_nh.param("num_marker_topics", numMarkerTopics, 5);
  private_nh.param("read_initial_poses", readInitialPoses, false);

  // get config files
  stringstream ss;
  ss << ros::package::getPath("rail_ceiling") << "/config/markers.yaml";
  string markerConfigFile;
  private_nh.param("markers_config", markerConfigFile, ss.str());
  ss.str("");
  ss << ros::package::getPath("rail_ceiling") << "/config/furniture_footprints.yaml";
  string furnitureConfigFile;
  private_nh.param("furniture_footprints_config", furnitureConfigFile, ss.str());

  readConfigFiles(markerConfigFile, furnitureConfigFile);
  lastPublishedPoses.resize(furnitureList.size());

  furnitureLayerPub = n.advertise<rail_ceiling::Obstacles>("furniture_layer/update_obstacles", 1);

  // subscribe to marker topics
  markerSubscribers.resize(numMarkerTopics);
  for (unsigned int i = 0; i < numMarkerTopics; i ++)
  {
    stringstream topicStream;
    topicStream << "ceiling_cam_tracker_" << i << "/ar_pose_marker";
    markerSubscribers[i] = n.subscribe(topicStream.str(), 1, &FurnitureTracker::markerCallback, this);
  }

  allPosesServer = n.advertiseService("furniture_tracker/get_all_poses", &FurnitureTracker::getAllPoses, this);
}

void FurnitureTracker::readConfigFiles(std::string markerConfigFile, std::string furnitureConfigFile)
{
  // parse the marker configuration file
#ifdef YAMLCPP_GT_0_5_0
  YAML::Node markerConfig = YAML::LoadFile(markerConfigFile);
  unsigned int id = 0;
  for (size_t i = 0; i < markerConfig.size(); i ++)
  {
    Furniture f;

    // load the ID and type
    f.id = id;
    id ++;
    f.type = markerConfig[i]["type"].as<string>();

    //optionally load
    if (readInitialPoses && markerConfig[i]["initial_pose"])
    {
      geometry_msgs::Pose2D initialPose;
      initialPose.x = markerConfig[i]["initial_pose"][0].as<double>();
      initialPose.y = markerConfig[i]["initial_pose"][1].as<double>();
      initialPose.theta = markerConfig[i]["initial_pose"][2].as<double>();
      furniturePoses.push_back(initialPose);
      ROS_INFO("Read initial position for furniture piece %lu", i);
    }
    else
    {
      furniturePoses.resize(furniturePoses.size() + 1);
    }

    // load marker information
    YAML::Node markers = markerConfig[i]["markers"];
    f.markers.resize(markers.size());
    for (size_t j = 0; j < markers.size(); j ++)
    {
      f.markers[j].id = markers[j]["id"].as<int>();
      f.markers[j].fid = f.id;
      f.markers[j].pose.x = markers[j]["x"].as<double>();
      f.markers[j].pose.y = markers[j]["y"].as<double>();
      f.markers[j].pose.theta = markers[j]["theta"].as<double>();
      if (f.markers[j].id >= markerList.size())
        markerList.resize(f.markers[j].id + 1);
      markerList[f.markers[j].id] = f.markers[j];
    }

    // store the furniture piece
    furnitureList.push_back(f);
  }
#else
  ifstream fin(markerConfigFile.c_str());
  YAML::Parser markerParser(fin);
  YAML::Node markerConfig;
  markerParser.GetNextDocument(markerConfig);
  unsigned int id = 0;
  for (size_t i = 0; i < markerConfig.size(); i ++)
  {
    Furniture f;

    // load the ID and type
    f.id = id;
    id ++;
    markerConfig[i]["type"] >> f.type;

    //optionally load
    if (readInitialPoses)
    {
      if (const YAML::Node *poseNode = markerConfig[i].FindValue("initial_pose"))
      {
        geometry_msgs::Pose2D initialPose;
        markerConfig[i]["initial_pose"][0] >> initialPose.x;
        markerConfig[i]["initial_pose"][1] >> initialPose.y;
        markerConfig[i]["initial_pose"][2] >> initialPose.theta;
        furniturePoses.push_back(initialPose);
        ROS_INFO("Read initial position for furniture piece %lu", i);
      }
      else
      {
        furniturePoses.resize(furniturePoses.size() + 1);
      }
    }
    else
    {
      furniturePoses.resize(furniturePoses.size() + 1);
    }

    // load marker information
    const YAML::Node &markers = markerConfig[i]["markers"];
    f.markers.resize(markers.size());
    for (size_t j = 0; j < markers.size(); j ++)
    {
      markers[j]["id"] >> f.markers[j].id;
      f.markers[j].fid = f.id;
      markers[j]["x"] >> f.markers[j].pose.x;
      markers[j]["y"] >> f.markers[j].pose.y;
      markers[j]["theta"] >> f.markers[j].pose.theta;
      if (f.markers[j].id >= markerList.size())
        markerList.resize(f.markers[j].id + 1);
      markerList[f.markers[j].id] = f.markers[j];
    }

    // store the furniture piece
    furnitureList.push_back(f);
  }
#endif

  ROS_INFO("Read marker configurations for %lu pieces of furniture.", furnitureList.size());

#ifdef YAMLCPP_GT_0_5_0
  // parse the furniture footprints configuration file
  YAML::Node furnitureConfig = YAML::LoadFile(furnitureConfigFile);
  for (size_t i = 0; i < furnitureConfig.size(); i ++)
  {
    FurnitureTransforms ft;
    ft.name = furnitureConfig[i]["name"].as<string>();
    if (furnitureConfig[i]["localization_footprint"])
    {
      YAML::Node polygons = furnitureConfig[i]["localization_footprint"];
      ft.localizationFootprint.resize(polygons.size());
      for (size_t j = 0; j < polygons.size(); j ++)
      {
        YAML::Node vertices = polygons[j]["polygon"];
        ft.localizationFootprint[j].points.resize(vertices.size());
        for (size_t k = 0; k < vertices.size(); k ++)
        {
          ft.localizationFootprint[j].points[k].x = vertices[k][0].as<float>();
          ft.localizationFootprint[j].points[k].y = vertices[k][1].as<float>();
          ft.localizationFootprint[j].points[k].z = 0.0;
        }
      }
    }
    if (furnitureConfig[i]["navigation_footprint"])
    {
      YAML::Node polygons = furnitureConfig[i]["navigation_footprint"];
      ft.navigationFootprint.resize(polygons.size());
      for (size_t j = 0; j < polygons.size(); j ++)
      {
        YAML::Node vertices = polygons[j]["polygon"];
        ft.navigationFootprint[j].points.resize(vertices.size());
        for (size_t k = 0; k < vertices.size(); k ++)
        {
          ft.navigationFootprint[j].points[k].x = vertices[k][0].as<float>();
          ft.navigationFootprint[j].points[k].y = vertices[k][1].as<float>();
          ft.navigationFootprint[j].points[k].z = 0.0;
        }
      }
    }
    footprintTransforms.push_back(ft);
  }
#else
  ifstream fin2(furnitureConfigFile.c_str());
  YAML::Parser furnitureParser(fin2);
  YAML::Node furnitureConfig;
  furnitureParser.GetNextDocument(furnitureConfig);
  for (size_t i = 0; i < furnitureConfig.size(); i ++)
  {
    FurnitureTransforms ft;
    furnitureConfig[i]["name"] >> ft.name;
    if (const YAML::Node *lfNode = furnitureConfig[i].FindValue("localization_footprint"))
    {
      const YAML::Node &polygons = furnitureConfig[i]["localization_footprint"];
      ft.localizationFootprint.resize(polygons.size());
      for (size_t j = 0; j < polygons.size(); j ++)
      {
        const YAML::Node &vertices = polygons[j]["polygon"];
        ft.localizationFootprint[j].points.resize(vertices.size());
        for (size_t k = 0; k < vertices.size(); k ++)
        {
          vertices[k][0] >> ft.localizationFootprint[j].points[k].x;
          vertices[k][1] >> ft.localizationFootprint[j].points[k].y;
          ft.localizationFootprint[j].points[k].z = 0.0;
        }
      }
    }
    if (const YAML::Node *nfNode = furnitureConfig[i].FindValue("navigation_footprint"))
    {
      const YAML::Node &polygons = furnitureConfig[i]["navigation_footprint"];
      ft.navigationFootprint.resize(polygons.size());
      for (size_t j = 0; j < polygons.size(); j ++)
      {
        const YAML::Node &vertices = polygons[j]["polygon"];
        ft.navigationFootprint[j].points.resize(vertices.size());
        for (size_t k = 0; k < vertices.size(); k ++)
        {
          vertices[k][0] >> ft.navigationFootprint[j].points[k].x;
          vertices[k][1] >> ft.navigationFootprint[j].points[k].y;
          ft.navigationFootprint[j].points[k].z = 0.0;
        }
      }
    }
    footprintTransforms.push_back(ft);
  }
#endif

  ROS_INFO("Read furniture footprints for:");
  for (unsigned int i = 0; i < footprintTransforms.size(); i ++)
  {
    ROS_INFO("%s", footprintTransforms[i].name.c_str());
  }
}

void FurnitureTracker::markerCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg)
{
  //update furniture poses based on new marker information
  for (unsigned int i = 0; i < msg->markers.size(); i ++)
  {
    //ignore if an unused marker is seen
    if (msg->markers[i].id >= markerList.size())
      continue;
    //uninitialized marker check
    if (markerList[msg->markers[i].id].id == 0)
      continue;

    Marker marker = markerList[msg->markers[i].id];
    unsigned int furnitureID = marker.fid;
    geometry_msgs::PoseStamped transformedPose;
    geometry_msgs::PoseStamped markerPose;
    markerPose = msg->markers[i].pose;
    markerPose.header.frame_id = msg->markers[i].header.frame_id;
    tfListener.transformPose("/map", markerPose, transformedPose);
    geometry_msgs::Pose2D furniturePose;
    float x = (float)transformedPose.pose.position.x;
    float y = (float)transformedPose.pose.position.y;
    float theta = (float)tf::getYaw(transformedPose.pose.orientation);
    float xt = -(float)marker.pose.x;
    float yt = -(float)marker.pose.y;
    float thetat = -(float)marker.pose.theta;
    furniturePose.x = x + xt*cos(theta + thetat) - yt*sin(theta + thetat);
    furniturePose.y = y + yt*cos(theta + thetat) + xt*sin(theta + thetat);
    furniturePose.theta = theta + thetat;

    if (furnitureID >= furniturePoses.size())
      furniturePoses.resize(furnitureID + 1);
    furniturePoses[furnitureID] = furniturePose;
  }
}

void FurnitureTracker::publishFurniturePoses()
{
  rail_ceiling::Obstacles obstacles;
  bool updatedFurniturePose = false;
  for (unsigned int i = 0; i < furnitureList.size(); i ++)
  {
    int index = furnitureList[i].id;
    string type = furnitureList[i].type;
    //check if a marker has been received for the listed piece of furniture
    if (index < furniturePoses.size())
    {
      //check if a pose has been calculated for the listed piece of furniture
      if (updated(furniturePoses[index]))
      {
        //check if a pose has ever been published for this piece of furniture
        if (updated(lastPublishedPoses[index]))
        {
          //publish pose if it has changed significantly
          if (sqrt(pow(lastPublishedPoses[index].x - furniturePoses[index].x, 2) + pow(lastPublishedPoses[index].y - furniturePoses[index].y, 2)) > POSITION_THRESHOLD
                || fabs(lastPublishedPoses[index].theta - furniturePoses[index].theta) > ANGULAR_THRESHOLD)
          {
            //DEBUG
            if (sqrt(pow(lastPublishedPoses[index].x - furniturePoses[index].x, 2) + pow(lastPublishedPoses[index].y - furniturePoses[index].y, 2)) > POSITION_THRESHOLD)
              ROS_INFO("Updated %s (id: %d) because of positional change.", type.c_str(), index);
            else if (fabs(lastPublishedPoses[index].theta - furniturePoses[index].theta) > ANGULAR_THRESHOLD)
              ROS_INFO("Updated %s (id: %d) because of angular change.", type.c_str(), index);

            updateMessages(index, type, &obstacles);
            updatedFurniturePose = true;
            lastPublishedPoses[index] = furniturePoses[index];
          }
        }
        else
        {
          //DEBUG
          ROS_INFO("Initial publish for %s (id: %d).", type.c_str(), index);

          //initial pose publish
          updateMessages(index, type, &obstacles);
          updatedFurniturePose = true;
          lastPublishedPoses[index] = furniturePoses[index];
        }
      }
    }
  }

  if (updatedFurniturePose)
  {
    furnitureLayerPub.publish(obstacles);
  }
}

bool FurnitureTracker::getAllPoses(rail_ceiling::GetAllObstacles::Request &req, rail_ceiling::GetAllObstacles::Response &res)
{
  rail_ceiling::Obstacles obstacles;
  for (unsigned int i = 0; i < furniturePoses.size(); i ++)
  {
    if (updated(furniturePoses[i]))
    {
      int index = furnitureList[i].id;
      string type = furnitureList[i].type;
      FurnitureTransforms footprints;
      bool footprintsSet = false;
      for (unsigned int j = 0; j < footprintTransforms.size(); j ++)
      {
        if (footprintTransforms[j].name.compare(type) == 0)
        {
          footprints = footprintTransforms[j];
          footprintsSet = true;
          break;
        }
      }
      if (!footprintsSet)
      {
        ROS_INFO("Could not find footprint information for furniture of type %s.", type.c_str());
      }
      else
      {
        fillFootprintInformation(index, footprints.localizationFootprint, &obstacles, true);
        fillFootprintInformation(index, footprints.navigationFootprint, &obstacles, false);
      }
    }
  }
  res.navigationObstacles = obstacles.navigationObstacles;
  res.localizationObstacles = obstacles.localizationObstacles;

  return true;
}

void FurnitureTracker::updateMessages(int index, std::string type, rail_ceiling::Obstacles *obstacles)
{
  //get footprints
  FurnitureTransforms footprints;
  bool footprintsSet = false;
  for (unsigned int j = 0; j < footprintTransforms.size(); j ++)
  {
    if (footprintTransforms[j].name.compare(type) == 0)
    {
      footprints = footprintTransforms[j];
      footprintsSet = true;
      break;
    }
  }
  if (!footprintsSet)
  {
    ROS_INFO("Could not find footprint information for furniture of type %s.", type.c_str());
    return;
  }

  //fill footprint polygons
  fillFootprintInformation(index, footprints.localizationFootprint, obstacles, true);
  fillFootprintInformation(index, footprints.navigationFootprint, obstacles, false);
}

void FurnitureTracker::fillFootprintInformation(int index, vector<geometry_msgs::Polygon> footprints, rail_ceiling::Obstacles *obstacles, bool isLocalization)
{
  if (!footprints.empty())
  {
    rail_ceiling::Obstacle obstacle;
    obstacle.polygons.clear();
    for (unsigned int j = 0; j < footprints.size(); j ++)
    {
      geometry_msgs::Polygon polygon;
      for (unsigned int k = 0; k < footprints[j].points.size(); k ++)
      {
        geometry_msgs::Point32 point;
        point.x = (float)(furniturePoses[index].x + footprints[j].points[k].x * cos(furniturePoses[index].theta)
            - footprints[j].points[k].y * sin(furniturePoses[index].theta));
        point.y = (float)(furniturePoses[index].y + footprints[j].points[k].x * sin(furniturePoses[index].theta)
            + footprints[j].points[k].y * cos(furniturePoses[index].theta));
        polygon.points.push_back(point);
      }
      obstacle.polygons.push_back(polygon);
    }
    obstacle.id = index;
    if (isLocalization)
      obstacles->localizationObstacles.push_back(obstacle);
    else
      obstacles->navigationObstacles.push_back(obstacle);
  }
}

bool FurnitureTracker::updated(geometry_msgs::Pose2D pose)
{
  return !(pose.x == 0 && pose.y == 0 && pose.theta == 0);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "furniture_tracker");

  FurnitureTracker ft;

  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    ft.publishFurniturePoses();
    ros::spinOnce();
    loop_rate.sleep();
  }
}

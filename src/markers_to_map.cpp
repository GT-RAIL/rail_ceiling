/*!
 * \markers_to_map.cpp
 * \brief places obstacles on a map at a location corresponding to an ar marker
 *
 * places obstacles on a map at a location corresponding to an ar marker
 *
 * \author Steven Kordell, WPI - spkordell@wpi.edu
 * \date July 10, 2014
 */

#include <rail_ceiling/markers_to_map.h>

using namespace std;

markers_to_map::markers_to_map()
{
  //a private handle for this ROS node
  ros::NodeHandle node("~");
  nh = node;

  //Read in parameters
  node.param<int>("camera_count", cameraCount, 1);
  node.param<double>("update_rate", updateRate, 2.0);
  node.param<double>("match_size_publish_period", matchSizePublishPeriod, 0.5);
  node.param<double>("match_data_publish_period", matchDataPublishPeriod, 15.0);
  node.param<double>("rolling_publish_period", rollingPublishPeriod, 0.5);
  node.param<double>("rolling_map_width", rollingMapWidth, 6.0);
  node.param<double>("rolling_map_height", rollingMapHeight, 6.0);
  node.param<bool>("load_static_map_from_file", loadStaticMapFromFile, false);
  node.param <string> ("static_map_yaml_file", staticMapYamlFile, "/");
  node.param<bool>("get_map_with_service", getMapWithService, false);
  node.param <string> ("layer_to_send_on_service_call", layerToSendOnServiceCall, "localization");
  node.param<bool>("dont_publish_while_navigating", dontPublishWhileNavigating, false);
  node.param<bool>("dont_publish_while_driving", dontPublishWhileDriving, false);
  node.param<double>("driving_timeout", drivingTimeout, 3.0);
  node.param <string> ("odom_frame_id", odomFrameId, "/odom");
  node.param <string> ("base_frame_id", baseFrameId, "/base_link");

  //initialize variables
  globalMapReceived = false;
  navigating = false;
  driving = false;

  markerDataIn = *(new vector<ar_track_alvar_msgs::AlvarMarkers::ConstPtr>(cameraCount));

  // create the ROS topics
  for (unsigned int i = 0; i < cameraCount; i++)
  {
    MarkerCallbackFunctor* marker_cback = new MarkerCallbackFunctor(&markerDataIn, i);
    markers_in.push_back(
        node.subscribe < ar_track_alvar_msgs::AlvarMarkers
            > ("ar_pose_marker_" + (boost::lexical_cast < string > (i)), 1, *marker_cback));
  }
  if (loadStaticMapFromFile)
  {
    globalMap = loadMapFromFile(staticMapYamlFile);
    globalMapReceived = true;
    //publish map meta data
    static_metadata_pub = node.advertise < nav_msgs::MapMetaData > ("map_metadata", 1, true);
    static_metadata_pub.publish(globalMap.info);
  }
  else
  {
    map_in = node.subscribe < nav_msgs::OccupancyGrid > ("map", 1, &markers_to_map::map_in_cback, this);
  }
  if (dontPublishWhileNavigating)
  {
    nav_goal_in = node.subscribe < geometry_msgs::PoseStamped > ("nav_goal", 10, &markers_to_map::nav_goal_cback, this);
    nav_goal_result = node.subscribe < move_base_msgs::MoveBaseActionResult
        > ("nav_goal_result", 10, &markers_to_map::nav_goal_result_cback, this);
  }
  if (dontPublishWhileDriving)
  {
    cmdVelTimer = node.createTimer(ros::Duration(drivingTimeout), &markers_to_map::cmdVelTimerCallback, this);
    cmdVelTimer.stop();
    cmd_vel_in = node.subscribe < geometry_msgs::Twist > ("cmd_vel", 1, &markers_to_map::cmd_vel_cback, this);
  }

  //create timers to publish different map layer types at different rates
  if (matchSizePublishPeriod)
  {
    matchSizeTimer = node.createTimer(ros::Duration(matchSizePublishPeriod),
                                      &markers_to_map::publishMatchSizeTimerCallback, this);
    matchSizeTimer.stop();
  }
  if (matchDataPublishPeriod)
  {
    matchDataTimer = node.createTimer(ros::Duration(matchDataPublishPeriod),
                                      &markers_to_map::publishMatchDataTimerCallback, this);
    matchDataTimer.stop();
  }
  if (rollingPublishPeriod)
  {
    rollingTimer = node.createTimer(ros::Duration(rollingPublishPeriod), &markers_to_map::publishRollingTimerCallback,
                                    this);
    rollingTimer.stop();
  }
  publishTimersStarted = false;

  if (getMapWithService)
  {
    static_map_service = node.advertiseService("static_map", &markers_to_map::staticMapServiceCallback, this);
  }

  ROS_INFO("Markers To Map Started");
}

bool markers_to_map::staticMapServiceCallback(nav_msgs::GetMap::Request &req, nav_msgs::GetMap::Response &res)
{
  for (unsigned int mapId = 0; mapId < mapLayers.size(); mapId++)
  {
    if (globalMapReceived && mapLayers[mapId]->mapData != NULL)
    {
      if (mapLayers[mapId]->name == layerToSendOnServiceCall)
      {
        mapLayers[mapId]->map->data = *(mapLayers[mapId]->mapData);
        res.map = *(mapLayers[mapId]->map);
        return true;
      }
    }
  }
  return false;
}

nav_msgs::OccupancyGrid markers_to_map::loadMapFromFile(const std::string& fname)
{
  std::string mapfname = "";
  double res;
  double origin[3];
  int negate;
  double occ_th, free_th;
  bool trinary = true;
  std::string frame_id = "map";
  nav_msgs::GetMap::Response map_resp_;
  std::ifstream fin(fname.c_str());
  if (fin.fail())
  {
    ROS_ERROR("markers_to_map could not open %s.", fname.c_str());
    exit(-1);
  }

#ifdef HAVE_NEW_YAMLCPP
//The document loading process changed in yaml-cpp 0.5.
  YAML::Node doc = YAML::Load(fin);
#else
  YAML::Parser parser(fin);
  YAML::Node doc;
  parser.GetNextDocument(doc);
#endif

  try
  {
    doc["resolution"] >> res;
  }
  catch (YAML::InvalidScalar)
  {
    ROS_ERROR("The map does not contain a resolution tag or it is invalid.");
    exit(-1);
  }
  try
  {
    doc["negate"] >> negate;
  }
  catch (YAML::InvalidScalar)
  {
    ROS_ERROR("The map does not contain a negate tag or it is invalid.");
    exit(-1);
  }
  try
  {
    doc["occupied_thresh"] >> occ_th;
  }
  catch (YAML::InvalidScalar)
  {
    ROS_ERROR("The map does not contain an occupied_thresh tag or it is invalid.");
    exit(-1);
  }
  try
  {
    doc["free_thresh"] >> free_th;
  }
  catch (YAML::InvalidScalar)
  {
    ROS_ERROR("The map does not contain a free_thresh tag or it is invalid.");
    exit(-1);
  }
  try
  {
    doc["trinary"] >> trinary;
  }
  catch (YAML::Exception)
  {
    ROS_DEBUG("The map does not contain a trinary tag or it is invalid... assuming true");
    trinary = true;
  }
  try
  {
    doc["origin"][0] >> origin[0];
    doc["origin"][1] >> origin[1];
    doc["origin"][2] >> origin[2];
  }
  catch (YAML::InvalidScalar)
  {
    ROS_ERROR("The map does not contain an origin tag or it is invalid.");
    exit(-1);
  }
  try
  {
    doc["image"] >> mapfname;
    if (mapfname.size() == 0)
    {
      ROS_ERROR("The image tag cannot be an empty string.");
      exit(-1);
    }
    if (mapfname[0] != '/')
    {
      char* fname_copy = strdup(fname.c_str());
      mapfname = std::string(dirname(fname_copy)) + '/' + mapfname;
      free(fname_copy);
    }
  }
  catch (YAML::InvalidScalar)
  {
    ROS_ERROR("The map does not contain an image tag or it is invalid.");
    exit(-1);
  }
  ROS_INFO("Loading map from image \"%s\"", mapfname.c_str());
  map_server::loadMapFromFile(&map_resp_, mapfname.c_str(), res, negate, occ_th, free_th, origin, trinary);
  map_resp_.map.info.map_load_time = ros::Time::now();
  map_resp_.map.header.frame_id = frame_id;
  map_resp_.map.header.stamp = ros::Time::now();
  ROS_INFO("Read a %d X %d map @ %.3lf m/cell", map_resp_.map.info.width, map_resp_.map.info.height,
           map_resp_.map.info.resolution);

  return map_resp_.map;
}

void markers_to_map::map_in_cback(const nav_msgs::OccupancyGrid::ConstPtr& map)
{
  globalMap = *map;
  globalMapReceived = true;
  ROS_INFO("Map Received");
}

float markers_to_map::round(float f, float prec)
{
  return (float)(floor(f * (1.0f / prec) + 0.5) / (1.0f / prec));
}

void markers_to_map::cmd_vel_cback(const geometry_msgs::Twist::ConstPtr& vel)
{
  cmdVelTimer.stop();
  driving = true;
  cmdVelTimer.start();
}

ar_track_alvar_msgs::AlvarMarkers* markers_to_map::mergeMarkerData()
{
//merge the marker data from all the cameras
  ar_track_alvar_msgs::AlvarMarkers* markers = new ar_track_alvar_msgs::AlvarMarkers();
  vector < ar_track_alvar_msgs::AlvarMarker > markerData;
  for (unsigned int camera = 0; camera < cameraCount; camera++)

  {
    if (markerDataIn[camera] == NULL)
    {
      continue; //No data from this camera yet. Skip it and move on.
    }
    for (unsigned int j = 0; j < markerDataIn[camera]->markers.size(); j ++)
    {
      bool contains = false;
      unsigned int k;
      for (k = 0; k < markerData.size(); k ++)
      {
        if (markerData[k].id == markerDataIn[camera]->markers[j].id)
        {
          contains = true;
          break;
        }
      }
      if (!contains)
      {
        markerData.push_back(markerDataIn[camera]->markers[j]);
      }
      else
      {
        //This marker was seen by more than 1 camera. Use information from whichever camera is closest to the marker.
        double distance = sqrt(
            pow(markerDataIn[camera]->markers[j].pose.pose.position.x, 2)
                + pow(markerDataIn[camera]->markers[j].pose.pose.position.y, 2)
                + pow(markerDataIn[camera]->markers[j].pose.pose.position.z, 2));

        double oldDistance = sqrt(
            pow(markerData[k].pose.pose.position.x, 2)
                + pow(markerData[k].pose.pose.position.y, 2)
                + pow(markerData[k].pose.pose.position.z, 2));
        if (distance < oldDistance)
        {
          markerData[k] = markerDataIn[camera]->markers[j];
        }
      }
    }
  }
  markers->markers = markerData;
  return markers;
}

void markers_to_map::initializeMaps()
{
//Initialize maps
  float globalOriginX = globalMap.info.origin.position.x;
  float globalOriginY = globalMap.info.origin.position.y;
  float globalWidth = globalMap.info.width;
  float globalHeight = globalMap.info.height;
  float globalResolution = globalMap.info.resolution;
  int rollingMapGridWidth = round(rollingMapWidth, globalResolution) / globalResolution;
  int rollingMapGridHeight = round(rollingMapHeight, globalResolution) / globalResolution;
  for (unsigned int mapId = 0; mapId < mapLayers.size(); mapId++)
  {
    //free any map and map data currently in memory
    delete mapLayers[mapId]->map;
    delete mapLayers[mapId]->mapData;

    mapLayers[mapId]->map = new nav_msgs::OccupancyGrid();
    mapLayers[mapId]->map->header.frame_id = globalMap.header.frame_id;
    mapLayers[mapId]->map->header.stamp = ros::Time::now();
    mapLayers[mapId]->map->info = globalMap.info;
    if (mapLayers[mapId]->mapType == MATCH_SIZE)
    {
      mapLayers[mapId]->mapData = new vector<signed char>(
          mapLayers[mapId]->map->info.width * mapLayers[mapId]->map->info.height);
    }
    else if (mapLayers[mapId]->mapType == MATCH_DATA)
    {
      mapLayers[mapId]->mapData = new vector<signed char>(globalMap.data);
    }
    else if (mapLayers[mapId]->mapType == ROLLING)
    {
      mapLayers[mapId]->mapData = new vector<signed char>(rollingMapGridWidth * rollingMapGridHeight);
      //get the global pose of the robot
      try
      {
        tf::StampedTransform transform;
        listener.lookupTransform(odomFrameId, baseFrameId, ros::Time(0), transform);
        mapLayers[mapId]->map->info.origin.position.x = round(
            transform.getOrigin().x() - rollingMapWidth / 2 + globalResolution / 2, globalResolution);
        mapLayers[mapId]->map->info.origin.position.y = round(
            transform.getOrigin().y() - rollingMapHeight / 2 + globalResolution / 2, globalResolution);
      }
      catch (tf::TransformException ex)
      {
        ROS_WARN("%s", ex.what());
      }
      mapLayers[mapId]->map->info.width = rollingMapGridWidth;
      mapLayers[mapId]->map->info.height = rollingMapGridHeight;
      mapLayers[mapId]->map->header.frame_id = odomFrameId;
    }
  }
}

void markers_to_map::updateMarkerMaps()
{
  static ar_track_alvar_msgs::AlvarMarkers* markers;
  if (globalMapReceived)
  {
    //prepare the marker data
    ar_track_alvar_msgs::AlvarMarkers* newMarkers = mergeMarkerData();
    if (markers != NULL)
    {
      //check the incoming marker data against the old set of marker data for markers which were entirely occluded
      for (unsigned int i = 0; i < markers->markers.size(); i++)
      {
        bool contains = false;
        for (unsigned int j = 0; j < newMarkers->markers.size(); j++)
        {
          if (markers->markers[i].id == newMarkers->markers[j].id)
          {
            contains = true;
            break;
          }
        }
        if (!contains)
        {
          //The marker was completely occluded, keep it on the map?

          //find the relevant bundle
          int bundleIndex = -1;
          for (unsigned int j = 0; j < bundles.size(); j++)
          {
            if (bundles[j]->getId() == markers->markers[i].id)
            {
              bundleIndex = j;
            }
          }
          if (bundleIndex == -1 || bundles[bundleIndex]->getKeepOnOcclusion())
          {
            //The marker was completely occluded and should be kept. Add it back to the list of markers in it's last known state.
            newMarkers->markers.push_back(markers->markers[i]);
          }
        }
      }
    }
    delete markers;
    markers = newMarkers;

    //prepare the maps
    initializeMaps();
    float globalResolution = globalMap.info.resolution;

    //Iterate over the detected marker bundles
    for (unsigned int i = 0; i < markers->markers.size(); i++)
    {
      //find the relevant bundle
      int bundleIndex = -1;
      for (unsigned int j = 0; j < bundles.size(); j++)
      {
        if (bundles[j]->getId() == markers->markers[i].id)
        {
          bundleIndex = j;
        }
      }
      if (bundleIndex == -1)
      {
        ROS_WARN("AR ID %d not found in list of bundles", markers->markers[i].id);
        continue;
      }

      for (unsigned int layerId = 0; layerId < bundles[bundleIndex]->getLayers()->size(); layerId++)
      {
        //iterate over every layer in this bundle
        try
        {
          //find the corresponding layers map
          unsigned int mapId;
          for (mapId = 0; mapId < mapLayers.size(); mapId++)
          {
            if (mapLayers[mapId]->name == bundles[bundleIndex]->getLayers()->at(layerId)->name)
            {
              break;
            }
          }
          //Find pose of ar_marker
          int xGrid;
          int yGrid;
          float angle;
          geometry_msgs::PoseStamped poseOut;
          if (mapLayers[mapId]->mapType == ROLLING)
          {
            //transform marker pose into odometry frame
            markers->markers[i].pose.header.frame_id = markers->markers[i].header.frame_id;
            listener.transformPose(odomFrameId, ros::Time(0), markers->markers[i].pose, markers->markers[i].header.frame_id, poseOut);
          }
          else
          {
            //transform marker pose into map frame
            markers->markers[i].pose.header.frame_id = markers->markers[i].header.frame_id;
            listener.transformPose(mapLayers[mapId]->map->header.frame_id, ros::Time(0), markers->markers[i].pose, markers->markers[i].pose.header.frame_id, poseOut);
          }
          //discretize to grid
          xGrid = round(poseOut.pose.position.x - mapLayers[mapId]->map->info.origin.position.x, globalResolution) / globalResolution;
          yGrid = round(poseOut.pose.position.y - mapLayers[mapId]->map->info.origin.position.y, globalResolution) / globalResolution;
          //extract the rotation angle
          tf::Quaternion q(poseOut.pose.orientation.x, poseOut.pose.orientation.y, poseOut.pose.orientation.z, poseOut.pose.orientation.w);
          double roll, pitch, yaw;
          tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
          angle = yaw;
          float rotCenterX = bundles[bundleIndex]->getMarkerX();
          float rotCenterY = bundles[bundleIndex]->getMarkerY();
          angle = angle + bundles[bundleIndex]->getMarkerYaw();

          //iterate over every polygon in this layer
          for (int poly = 0; poly < bundles[bundleIndex]->getLayers()->at(layerId)->footprint.size(); poly++)
          {
            //transform the polygon footprint
            geometry_msgs::PolygonStamped transformedFootprint;
            transformedFootprint.header.frame_id = bundles[bundleIndex]->getLayers()->at(layerId)->footprint[poly]->header.frame_id;
            for (int pt = 0; pt < bundles[bundleIndex]->getLayers()->at(layerId)->footprint[poly]->polygon.points.size(); pt++)
            {
              geometry_msgs::Point32 point = bundles[bundleIndex]->getLayers()->at(layerId)->footprint[poly]->polygon.points[pt];
              //translate by center of rotation
              point.x += rotCenterX;
              point.y += rotCenterY;
              //rotate by desired angle
              float x = point.x * cos(angle) - point.y * sin(angle);
              float y = point.x * sin(angle) + point.y * cos(angle);
              point.x = x;
              point.y = y;
              //translate back to origin
              point.x -= rotCenterX;
              point.y -= rotCenterY;
              transformedFootprint.polygon.points.push_back(point);
            }

            //find bounding box of polygon footprint
            float minX = numeric_limits<float>::max();
            float maxX = -numeric_limits<float>::max();
            float minY = numeric_limits<float>::max();
            float maxY = -numeric_limits<float>::max();
            for (unsigned int pt = 0; pt < transformedFootprint.polygon.points.size(); pt++)
            {
              float x = transformedFootprint.polygon.points[pt].x;
              float y = transformedFootprint.polygon.points[pt].y;
              if (x < minX)
                minX = x;
              if (x > maxX)
                maxX = x;
              if (y < minY)
                minY = y;
              if (y > maxY)
                maxY = y;
            }
            maxX = round(maxX, globalResolution) / globalResolution;
            minX = round(minX, globalResolution) / globalResolution;
            maxY = round(maxY, globalResolution) / globalResolution;
            minY = round(minY, globalResolution) / globalResolution;
            int width = abs(maxX) - minX;
            int height = abs(maxY) - minY;

            //rasterize polygon footprint
            cv::Mat obsMat = cv::Mat::zeros(height + 2, width + 2, CV_8U); //have to extend the width and height a little to prevent line trimming
            int lineType = 8; // 8-connected line
            cv::Point obsPoints[transformedFootprint.polygon.points.size()];
            for (unsigned int pt = 0; pt < transformedFootprint.polygon.points.size(); pt++)
            {
              int x = round(transformedFootprint.polygon.points[pt].x, globalResolution) / globalResolution;
              x -= minX;
              int y = round(transformedFootprint.polygon.points[pt].y, globalResolution) / globalResolution;
              y -= minY;
              obsPoints[pt] = cv::Point(x, y);
            }
            const cv::Point* ppt[1] = {obsPoints};
            int npt[] = {transformedFootprint.polygon.points.size()};

            if (bundles[bundleIndex]->getLayers()->at(layerId)->fillPolygon)
            {
              cv::fillPoly(obsMat, ppt, npt, 1, 255, lineType);
            }
            else
            {
              cv::polylines(obsMat, ppt, npt, 1, true, 255, 1, lineType);
            }

            //draw obstacle on map
            int xOffset = minX + round(rotCenterX, globalResolution) / globalResolution;
            int yOffset = minY + round(rotCenterY, globalResolution) / globalResolution;
            for (unsigned int x = 0; x < obsMat.cols; x++)
            {
              for (unsigned int y = 0; y < obsMat.rows; y++)
              {
                if (obsMat.at < uchar > (y, x) > 128)
                {
                  //check if point is within  map limits
                  if ((xGrid + x + xOffset) > 0 && (xGrid + x + xOffset) < mapLayers[mapId]->map->info.width
                      && (yGrid + y + yOffset) > 0 && (yGrid + y + yOffset) < mapLayers[mapId]->map->info.height)
                  {
                    //draw on map
                    mapLayers[mapId]->mapData->at(
                        (xGrid + x + xOffset) + (yGrid + y + yOffset) * mapLayers[mapId]->map->info.width) = 100;
                  }
                }
              }
            }
          }
        }
        catch (tf::TransformException ex)
        {
          ROS_WARN("%s", ex.what());
        }
      }
    }
    if (!publishTimersStarted)
    {
      //Publish the maps immediately and start the timers for regular publishing
      publishMatchSizeTimerCallback(*(new ros::TimerEvent));
      publishMatchDataTimerCallback(*(new ros::TimerEvent));
      publishRollingTimerCallback(*(new ros::TimerEvent));
      if (matchSizePublishPeriod)
      {
        matchSizeTimer.start();
      }
      if (matchDataPublishPeriod)
      {
        matchDataTimer.start();
      }
      if (rollingPublishPeriod)
      {
        rollingTimer.start();
      }
      publishTimersStarted = true;
    }
  }
}

void markers_to_map::initializeLayers()
{
  for (unsigned int i = 0; i < bundles.size(); i++)
  {
    for (unsigned int j = 0; j < bundles[i]->getLayers()->size(); j++)
    {
      bool contains = false;
      for (unsigned int mapId = 0; mapId < mapLayers.size(); mapId++)
      {
        if (mapLayers[mapId]->name == bundles[i]->getLayers()->at(j)->name)
        {
          contains = true;
          break;
        }
      }
      if (!contains)
      {
        layer_info_t* layer = new layer_info_t();
        layer->name = bundles[i]->getLayers()->at(j)->name;
        layer->mapType = bundles[i]->getLayers()->at(j)->mapType;
        layer->publisher = nh.advertise < nav_msgs::OccupancyGrid
            > ("ar_" + bundles[i]->getLayers()->at(j)->name + "_map", 1, true);
        mapLayers.push_back(layer);
        ROS_INFO("Found layer: %s", layer->name.c_str());
      }
    }
  }
}

void markers_to_map::nav_goal_cback(const geometry_msgs::PoseStamped::ConstPtr& nav_goal)
{
  navigating = true;
}

void markers_to_map::nav_goal_result_cback(const move_base_msgs::MoveBaseActionResult::ConstPtr& result)
{
  if (result->status.text != "This goal was canceled because another goal was recieved by the simple action server")
  {
    navigating = false;
  }
}

void markers_to_map::publishMatchSizeTimerCallback(const ros::TimerEvent&)
{
  for (unsigned int mapId = 0; mapId < mapLayers.size(); mapId++)
  {
    if (mapLayers[mapId]->mapType == MATCH_SIZE)
    {
      mapLayers[mapId]->map->data = *(mapLayers[mapId]->mapData);
      mapLayers[mapId]->publisher.publish(*(mapLayers[mapId]->map));
    }
  }
}

void markers_to_map::publishMatchDataTimerCallback(const ros::TimerEvent&)
{
  for (unsigned int mapId = 0; mapId < mapLayers.size(); mapId++)
  {
    if (mapLayers[mapId]->mapType == MATCH_DATA)
    {
      if (dontPublishWhileNavigating)
      {
        ros::Rate loop_rate(getUpdateRate());
        while (navigating && ros::ok())
        {
          ros::spinOnce(); //wait for navigation to finish before publishing localization map
          updateMarkerMaps();
          loop_rate.sleep();
        }
      }
      if (dontPublishWhileDriving)
      {
        ros::Rate loop_rate(getUpdateRate());
        while (driving && ros::ok())
        {
          ros::spinOnce(); //wait for driving to finish before publishing localization map
          updateMarkerMaps();
          loop_rate.sleep();
        }
      }
      mapLayers[mapId]->map->data = *(mapLayers[mapId]->mapData);
      mapLayers[mapId]->publisher.publish(*(mapLayers[mapId]->map));
    }
  }
}

void markers_to_map::publishRollingTimerCallback(const ros::TimerEvent&)
{
  for (unsigned int mapId = 0; mapId < mapLayers.size(); mapId++)
  {
    if (mapLayers[mapId]->mapType == ROLLING)
    {
      mapLayers[mapId]->map->data = *(mapLayers[mapId]->mapData);
      mapLayers[mapId]->publisher.publish(*(mapLayers[mapId]->map));
    }
  }
}

void markers_to_map::cmdVelTimerCallback(const ros::TimerEvent&)
{
  driving = false;
  cmdVelTimer.stop();
}

void markers_to_map::addBundle(Bundle* bundle)
{
  bundles.push_back(bundle);
}

Bundle* markers_to_map::getBundle(int index)
{
  return bundles[index];
}

double markers_to_map::getUpdateRate()
{
  return updateRate;
}

int main(int argc, char **argv)
{
//initialize ROS and the node
  ros::init(argc, argv, "markers_to_map");

//initialize the converter
  markers_to_map converter;

  ros::Rate loop_rate(converter.getUpdateRate());

//Parse bundle files provided as input arguments
  for (int arg = 1; arg < argc; arg++)
  {
    Bundle* bundle = new Bundle();
    if (bundle->parseBundleFootprint(argv[arg]))
      converter.addBundle(bundle);
  }

//create the output map topics for each map layer
  converter.initializeLayers();

//short delay for cleaner startup
  ros::Duration(3.0).sleep();

  while (ros::ok())
  {
    ros::spinOnce();
    converter.updateMarkerMaps();
    loop_rate.sleep();
  }

  return EXIT_SUCCESS;
}

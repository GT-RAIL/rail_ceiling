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
  node.param < string > ("odom_frame_id", odomFrameId, "/odom");
  node.param < string > ("base_frame_id", baseFrameId, "/base_link");

  //initialize variables
  globalMapReceived = false;
  markerDataIn = *(new vector<ar_track_alvar::AlvarMarkers::ConstPtr>(cameraCount));
  markerVisDataIn = *(new vector<vector<visualization_msgs::Marker::ConstPtr> >(cameraCount));

  // create the ROS topics
  for (unsigned int i = 0; i < cameraCount; i++)
  {
    MarkerCallbackFunctor* marker_cback = new MarkerCallbackFunctor(&markerDataIn, i);
    MarkerVisCallbackFunctor* vis_marker_cback = new MarkerVisCallbackFunctor(&markerVisDataIn, i);
    markers_in.push_back(
        node.subscribe < ar_track_alvar::AlvarMarkers
            > ("ar_pose_marker_" + (boost::lexical_cast < string > (i)), 1, *marker_cback));
    vis_markers_in.push_back(
        node.subscribe < visualization_msgs::Marker
            > ("ar_vis_marker_" + (boost::lexical_cast < string > (i)), 1, *vis_marker_cback));
  }
  map_in = node.subscribe < nav_msgs::OccupancyGrid > ("map", 1, &markers_to_map::map_in_cback, this);

  //create timers to publish different map layer types at different rates
  matchSizeTimer = node.createTimer(ros::Duration(matchSizePublishPeriod),
                                    &markers_to_map::publishMatchSizeTimerCallback, this);
  matchSizeTimer.stop();
  matchDataTimer = node.createTimer(ros::Duration(matchDataPublishPeriod),
                                    &markers_to_map::publishMatchDataTimerCallback, this);
  matchDataTimer.stop();
  rollingTimer = node.createTimer(ros::Duration(rollingPublishPeriod), &markers_to_map::publishRollingTimerCallback,
                                  this);
  rollingTimer.stop();
  publishTimersStarted = false;

  ROS_INFO("Markers To Map Started");
}

float markers_to_map::round(float f, float prec)
{
  return (float)(floor(f * (1.0f / prec) + 0.5) / (1.0f / prec));
}

void markers_to_map::map_in_cback(const nav_msgs::OccupancyGrid::ConstPtr& map)
{
  globalMap = *map;
  globalMapReceived = true;
  ROS_INFO("Map Received");
}

ar_track_alvar::AlvarMarkers* markers_to_map::mergeMarkerData()
{
  //merge the marker data from all the cameras
  ar_track_alvar::AlvarMarkers* markers = new ar_track_alvar::AlvarMarkers();
  vector < ar_track_alvar::AlvarMarker > markerData;
  vector<int> associatedCameras;
  for (unsigned int camera = 0; camera < markerDataIn.size(); camera++)
  {
    for (unsigned int j = 0; j < markerDataIn[camera]->markers.size(); j++)
    {
      bool contains = false;
      unsigned int k;
      for (k = 0; k < markerData.size(); k++)
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
        associatedCameras.push_back(camera);
      }
      else
      {
        //this marker was seen by more than 1 camera. Use information from whichever camera is closest to the marker.
        double distance;
        //find the pose of this marker with respect to its camera
        for (unsigned int markIndex = 0; markIndex < markerVisDataIn[camera].size(); markIndex++)
        {
          if (markerVisDataIn[camera].at(markIndex)->id == markerDataIn[camera]->markers[j].id)
          {
            distance = sqrt(
                pow(markerVisDataIn[camera].at(markIndex)->pose.position.x, 2)
                    + pow(markerVisDataIn[camera].at(markIndex)->pose.position.y, 2)
                    + pow(markerVisDataIn[camera].at(markIndex)->pose.position.z, 2));
            break;
          }
        }
        //find the pose of the current marker in the list
        for (unsigned int markIndex = 0; markIndex < markerVisDataIn[associatedCameras[k]].size(); markIndex++)
        {
          if (markerVisDataIn[associatedCameras[k]].at(markIndex)->id == markerData[k].id)
          {
            double oldDistance = sqrt(
                pow(markerVisDataIn[associatedCameras[k]].at(markIndex)->pose.position.x, 2)
                    + pow(markerVisDataIn[associatedCameras[k]].at(markIndex)->pose.position.y, 2)
                    + pow(markerVisDataIn[associatedCameras[k]].at(markIndex)->pose.position.z, 2));
            //use the new one marker if it is closer to the camera than the old marker (markers closer to the camera will be more accurate)
            if (distance < oldDistance)
            {
              markerData[k] = markerDataIn[camera]->markers[j];
              associatedCameras[k] = camera;
            }
            break;
          }
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
    mapLayers[mapId]->map->header.frame_id = "map";
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
  if (globalMapReceived)
  {
    //ensure every camera is publishing before trying to access the data
    for (unsigned int camera = 0; camera < markerDataIn.size(); camera++)
    {
      if (markerDataIn[camera] == NULL)
      {
        return;
      }
    }
    ar_track_alvar::AlvarMarkers* markers = mergeMarkerData();
    initializeMaps();
    float globalResolution = globalMap.info.resolution;

    //Iterate over the detected marker bundles
    for (int i = 0; i < markers->markers.size(); i++)
    {
      //find the relevant bundle
      int bundleIndex = -1;
      for (int j = 0; j < bundles.size(); j++)
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

      try
      {
        //iterate over every layer in this bundle
        for (unsigned int layerId = 0; layerId < bundles[bundleIndex]->getLayers()->size(); layerId++)
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
          //Find transform to ar_marker
          //tf::StampedTransform transform;
          int xGrid;
          int yGrid;
          float angle;
          if (mapLayers[mapId]->mapType != ROLLING)
          {
            /*
             listener.lookupTransform("/map", "/ar_marker_" + (boost::lexical_cast < string > (markers->markers[i].id)),
             ros::Time(0), transform);
             */
            xGrid = round(markers->markers[i].pose.pose.position.x - mapLayers[mapId]->map->info.origin.position.x,
                          globalResolution) / globalResolution;
            yGrid = round(markers->markers[i].pose.pose.position.y - mapLayers[mapId]->map->info.origin.position.y,
                          globalResolution) / globalResolution;
            //extract the rotation angle
            tf::Quaternion q(markers->markers[i].pose.pose.orientation.x, markers->markers[i].pose.pose.orientation.y,
                             markers->markers[i].pose.pose.orientation.z, markers->markers[i].pose.pose.orientation.w);
            double roll, pitch, yaw;
            tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
            angle = yaw;
          }
          else
          {
            /*
             listener.lookupTransform(odomFrameId,
             "/ar_marker_" + (boost::lexical_cast < string > (markers->markers[i].id)),
             ros::Time(0), transform);
             */
            geometry_msgs::PoseStamped poseOut;
            listener.transformPose(odomFrameId, ros::Time(0), markers->markers[i].pose, "map", poseOut);

            xGrid = round(poseOut.pose.position.x - mapLayers[mapId]->map->info.origin.position.x, globalResolution)
                / globalResolution;
            yGrid = round(poseOut.pose.position.y - mapLayers[mapId]->map->info.origin.position.y, globalResolution)
                / globalResolution;
            //extract the rotation angle
            tf::Quaternion q(poseOut.pose.orientation.x, poseOut.pose.orientation.y, poseOut.pose.orientation.z,
                             poseOut.pose.orientation.w);
            double roll, pitch, yaw;
            tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
            angle = yaw;
          }
          float rotCenterX = bundles[bundleIndex]->getMarkerX();
          float rotCenterY = bundles[bundleIndex]->getMarkerY();
          angle = angle + bundles[bundleIndex]->getMarkerYaw();
          /*
           int xGrid = round(transform.getOrigin().x() - mapLayers[mapId]->map->info.origin.position.x, globalResolution)
           / globalResolution;
           int yGrid = round(transform.getOrigin().y() - mapLayers[mapId]->map->info.origin.position.y, globalResolution)
           / globalResolution;
           //extract the rotation angle
           tf::Quaternion q(transform.getRotation().x(), transform.getRotation().y(), transform.getRotation().z(),
           transform.getRotation().w());
           double roll, pitch, yaw;
           tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
           float angle = yaw;

           */

          //iterate over every polygon in this layer
          for (int poly = 0; poly < bundles[bundleIndex]->getLayers()->at(layerId)->footprint.size(); poly++)
          {
            //transform the polygon footprint
            geometry_msgs::PolygonStamped transformedFootprint;
            transformedFootprint.header.frame_id =
                bundles[bundleIndex]->getLayers()->at(layerId)->footprint[poly]->header.frame_id;
            for (int pt = 0;
                pt < bundles[bundleIndex]->getLayers()->at(layerId)->footprint[poly]->polygon.points.size(); pt++)
            {
              geometry_msgs::Point32 point =
                  bundles[bundleIndex]->getLayers()->at(layerId)->footprint[poly]->polygon.points[pt];
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
            cv::Mat obsMat = cv::Mat::zeros(height, width, CV_8U);
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
            cv::fillPoly(obsMat, ppt, npt, 1, 255, lineType);

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
      }
      catch (tf::TransformException ex)
      {
        ROS_WARN("%s", ex.what());
      }
    }
    if (!publishTimersStarted)
    {
      //Publish the maps immediately
      publishMatchSizeTimerCallback(*(new ros::TimerEvent));
      publishMatchDataTimerCallback(*(new ros::TimerEvent));
      publishRollingTimerCallback(*(new ros::TimerEvent));
      //start the timers to publish at interval times from now on
      matchSizeTimer.start();
      matchDataTimer.start();
      rollingTimer.start();
      publishTimersStarted = true;
    }

    //free used memory
    delete markers;
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
  // initialize ROS and the node
  ros::init(argc, argv, "markers_to_map");

  // initialize the converter
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

  while (ros::ok())
  {
    ros::spinOnce();
    converter.updateMarkerMaps();
    loop_rate.sleep();
  }

  return EXIT_SUCCESS;
}

/*!
 * \markers_to_map.cpp
 * \brief places obstacles on a map at a location corresponding to an ar marker
 *
 * places obstacles on a map at a location corresponding to an ar marker
 *
 * \author Steven Kordell, WPI - spkordell@wpi.edu
 * \date June 25, 2014
 */

#include <rail_ceiling/markers_to_map.h>

using namespace std;

markers_to_map::markers_to_map()
{
  //a private handle for this ROS node
  ros::NodeHandle node("~");
  nh = node;

  //initialize variables
  globalMapReceived = false;

  //Read in the update rate
  node.param<double>("update_rate", updateRate, 0.2);

  // create the ROS topics
  markers_in = node.subscribe < ar_track_alvar::AlvarMarkers
      > ("ar_pose_marker", 1, &markers_to_map::markers_cback, this);
  map_in = node.subscribe < nav_msgs::OccupancyGrid > ("map", 1, &markers_to_map::map_in_cback, this);
  footprint_out = node.advertise < geometry_msgs::PolygonStamped > ("bundle_footprint", 1);

  ROS_INFO("Markers To Map Started");
}

float markers_to_map::round(float f, float prec)
{
  return (float)(floor(f * (1.0f / prec) + 0.5) / (1.0f / prec));
}

float markers_to_map::min(float a, float b)
{
  if (a <= b)
    return a;
  return b;
}

float markers_to_map::max(float a, float b)
{
  if (a >= b)
    return a;
  return b;
}

void markers_to_map::map_in_cback(const nav_msgs::OccupancyGrid::ConstPtr& map)
{
  globalMap = *map;
  globalMapReceived = true;
  ROS_INFO("Map Received");
}

void markers_to_map::markers_cback(const ar_track_alvar::AlvarMarkers::ConstPtr& markers)
{
  if (globalMapReceived)
  {
    //Initialize maps
    vector<signed char> globalMapData = globalMap.data;
    for (unsigned int mapId = 0; mapId < mapLayers.size(); mapId++) {
      mapLayers[mapId]->map = new nav_msgs::OccupancyGrid();
      mapLayers[mapId]->map->header.frame_id = "ar_map";
      mapLayers[mapId]->map->header.stamp = ros::Time::now();
      mapLayers[mapId]->map->info = globalMap.info;
      if (mapLayers[mapId]->mapType == MATCH_SIZE) {
        mapLayers[mapId]->mapData = new vector<signed char>(mapLayers[mapId]->map->info.width * mapLayers[mapId]->map->info.height);
      } else if (mapLayers[mapId]->mapType == MATCH_DATA) {
        mapLayers[mapId]->mapData = &globalMapData;
      }
    }
    float globalOriginX = globalMap.info.origin.position.x;
    float globalOriginY = globalMap.info.origin.position.y;
    float globalWidth = globalMap.info.width;
    float globalHeight = globalMap.info.height;
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
        //Find transform to ar_marker
        tf::StampedTransform transform;
        listener.lookupTransform("/ar_map", "/ar_marker_" + (boost::lexical_cast < string > (markers->markers[i].id)),
                                 ros::Time(0), transform);
        int xGrid = round(transform.getOrigin().x() - globalOriginX, globalResolution)
            / globalResolution;
        int yGrid = round(transform.getOrigin().y() - globalOriginY, globalResolution)
            / globalResolution;

        //extract the rotation angle
        tf::Quaternion q(transform.getRotation().x(), transform.getRotation().y(), transform.getRotation().z(),
                         transform.getRotation().w());
        double roll, pitch, yaw;
        tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
        float angle = yaw;

        for (unsigned int layerId = 0; layerId < bundles[bundleIndex]->getLayers()->size(); layerId++) {

          //find the layers map
          unsigned int mapId;
          for (mapId = 0; mapId < mapLayers.size(); mapId++) {
            if (mapLayers[mapId]->name == bundles[bundleIndex]->getLayers()->at(layerId)->name) {
              break;
            }
          }

          //transform the polygon footprint
          float rotCenterX = bundles[bundleIndex]->getMarkerX();
          float rotCenterY = bundles[bundleIndex]->getMarkerY();
          angle = angle + bundles[bundleIndex]->getMarkerYaw();

          for (int poly = 0; poly < bundles[bundleIndex]->getLayers()->at(layerId)->footprint.size(); poly++) {

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
            //footprint_out.publish(transformedFootprint);

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
                if (obsMat.at < uchar > (y, x) > 128) {
                  mapLayers[mapId]->mapData->at((xGrid + x + xOffset) + (yGrid + y + yOffset) * globalWidth) = 100;
                }
              }
            }
          }
        }
      }
      catch (tf::TransformException ex)
      {
        ROS_ERROR("%s", ex.what());
      }
    }

    //publish all the maps
    for (unsigned int mapId = 0; mapId < mapLayers.size(); mapId++) {
      mapLayers[mapId]->map->data = *(mapLayers[mapId]->mapData);
      mapLayers[mapId]->publisher.publish(*(mapLayers[mapId]->map));
    }
  }
}

void markers_to_map::createMapTopics() {
  for (unsigned int i = 0; i < bundles.size(); i++) {
    for (unsigned int j = 0; j < bundles[i]->getLayers()->size(); j++) {
      bool contains = false;
      for (unsigned int mapId = 0; mapId < mapLayers.size(); mapId++) {
        if (mapLayers[mapId]->name == bundles[i]->getLayers()->at(j)->name) {
          contains = true;
          break;
        }
      }
      if (!contains) {
        layer_info_t* layer = new layer_info_t();;
        layer->name = bundles[i]->getLayers()->at(j)->name;
        layer->mapType = bundles[i]->getLayers()->at(j)->mapType;
        layer->publisher = nh.advertise < nav_msgs::OccupancyGrid > ("ar_"+bundles[i]->getLayers()->at(j)->name+"_map", 1);
        mapLayers.push_back(layer);
        ROS_INFO("Found layer: %s", layer->name.c_str());
      }
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
  converter.createMapTopics();

  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return EXIT_SUCCESS;
}

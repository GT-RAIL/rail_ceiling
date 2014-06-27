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

  //initialize variables
  mapReceived = false;

  //Read in the update rate
  node.param<double>("update_rate", updateRate, 0.2);

  // create the ROS topics
  markers_in = node.subscribe < ar_track_alvar::AlvarMarkers
      > ("ar_pose_marker", 1, &markers_to_map::markers_cback, this);
  map_in = node.subscribe < nav_msgs::OccupancyGrid > ("map", 1, &markers_to_map::map_in_cback, this);
  map_out = node.advertise < nav_msgs::OccupancyGrid > ("marker_map", 1);

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
  mapReceived = true;
  ROS_INFO("Map Received");
}

void markers_to_map::markers_cback(const ar_track_alvar::AlvarMarkers::ConstPtr& markers)
{
  if (mapReceived)
  {
    //Initialize map object
    nav_msgs::OccupancyGrid map;
    map.header.frame_id = "ar_map";
    map.header.stamp = ros::Time::now();
    map.info = globalMap.info;
    vector<signed char> mapData(map.info.width * map.info.height);
    //fill(mapData.begin(), mapData.end(), -1);

    //Iterate over every marker bundle
    for (int i = 0; i < markers->markers.size(); i++)
    {
      float xAbsLength = 0;
      float yAbsLength = 0;
      float markerRadius = 0;
      //find the relevant bundle
      for (int j = 0; j < bundles.size(); j++)
      {
        if (bundles[j]->getId() == markers->markers[i].id)
        {
          markerRadius = sqrt(2 * pow(bundles[j]->getMarkerSize() / 2, 2));
          xAbsLength = bundles[j]->getBundleWidth() + 2 * markerRadius;
          yAbsLength = bundles[j]->getBundleHeight() + 2 * markerRadius;
        }
      }
      if (xAbsLength == 0 || yAbsLength == 0)
      {
        ROS_ERROR("AR ID not found in list of bundles");
        continue;
      }

      try
      {
        //Find transform and discretize sizes to grid
        tf::StampedTransform transform;
        listener.lookupTransform("/ar_map", "/ar_marker_" + (boost::lexical_cast < string > (markers->markers[i].id)),
                                 ros::Time(0), transform);
        float xAbs = round(transform.getOrigin().x() - map.info.origin.position.x, map.info.resolution);
        float yAbs = round(transform.getOrigin().y() - map.info.origin.position.y, map.info.resolution);
        int xGrid = xAbs / map.info.resolution;
        int yGrid = yAbs / map.info.resolution;
        int xGridLength = round(xAbsLength, map.info.resolution) / map.info.resolution;
        int yGridLength = round(yAbsLength, map.info.resolution) / map.info.resolution;
        int markerRadiusGridX = round(markerRadius, map.info.resolution) / map.info.resolution;
        int markerRadiusGridY = round(markerRadius, map.info.resolution) / map.info.resolution;

        //Create the obstacle in its own grid
        nav_msgs::OccupancyGrid obstacle;
        vector<signed char> obstacleData(xGridLength * yGridLength);
        fill(obstacleData.begin(), obstacleData.end(), 100);
        obstacle.info.width = xGridLength;
        obstacle.info.height = yGridLength;

        //Find the needed rotation angle
        tf::Quaternion q(transform.getRotation().x(), transform.getRotation().y(), transform.getRotation().z(),
                         transform.getRotation().w());
        double roll, pitch, yaw;
        tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
        float angle = yaw;
        float cosine = (float)cos(angle);
        float sine = (float)sin(angle);

        //Calculate dimensions of rotated obstacles bounding box
        float height = (float)obstacle.info.height;
        float width = (float)obstacle.info.width;
        float Point1x = (-height * sine);
        float Point1y = (height * cosine);
        float Point2x = (width * cosine - height * sine);
        float Point2y = (height * cosine + width * sine);
        float Point3x = (width * cosine);
        float Point3y = (width * sine);
        float minx = min(0, min(Point1x, min(Point2x, Point3x)));
        float miny = min(0, min(Point1y, min(Point2y, Point3y)));
        float maxx = max(Point1x, max(Point2x, Point3x));
        float maxy = max(Point1y, max(Point2y, Point3y));
        int DestWidth = (int)ceil(fabs(maxx) - minx);
        int DestHeight = (int)ceil(fabs(maxy) - miny);

        //Calculate offset from bounding box for drawing on map based on the quadrant the obstacle was rotated into
        int xOffset;
        int yOffset;
        if ((angle >= 0 && angle <= (PI / 2) + 0.001) || (angle >= -2 * PI - 0.001 && angle < -3 * PI / 2))
        {
          xOffset = -Point3x + Point1x;
          yOffset = -Point3y;
        }
        else if ((angle > PI / 2 && angle <= PI + 0.001) || (angle >= -3 * PI / 2 - 0.001 && angle < -PI))
        {
          xOffset = Point1x;
          yOffset = -Point3y + Point1y;
          markerRadiusGridX *= -2;
        }
        else if ((angle > PI && angle <= (3 * PI / 2) + 0.001) || (angle >= -PI - 0.001 && angle < -PI / 2))
        {
          xOffset = 0;
          yOffset = Point1y;
          markerRadiusGridX *= -1;
          markerRadiusGridY *= -1;
        }
        else if ((angle > (3 * PI / 2) && angle <= (2 * PI) + 0.001) || (angle >= -PI / 2 - 0.001 && angle < 0))
        {
          xOffset = -Point3x;
          yOffset = 0;
          markerRadiusGridY *= -1;
        }

        //Rotate every point on the obstacle and draw it on the map
        for (int x = 0; x < DestWidth; x++)
        {
          for (int y = 0; y < DestHeight; y++)
          {
            int SrcX = (int)((x + minx) * cosine + (y + miny) * sine);
            int SrcY = (int)((y + miny) * cosine - (x + minx) * sine);
            if (SrcX >= 0 && SrcX < obstacle.info.width && SrcY >= 0 && SrcY < obstacle.info.height)
            {
              mapData[(xGrid + x + xOffset + markerRadiusGridX)
                  + (yGrid + y + yOffset + markerRadiusGridY) * map.info.width] = obstacleData[SrcX
                  + SrcY * obstacle.info.width];
            }
          }
        }
      }
      catch (tf::TransformException ex)
      {
        ROS_ERROR("%s", ex.what());
      }
    }
    //publish the map
    map.data = mapData;
    map_out.publish(map);
  }
}

void markers_to_map::addBundle(Bundle* bundle)
{
  bundles.push_back(bundle);
}

double markers_to_map::getUpdateRate() {
  return updateRate;
}

int main(int argc, char **argv)
{
  // initialize ROS and the node
  ros::init(argc, argv, "markers_to_map");

  // initialize the converter
  markers_to_map converter;

  //Parse bundle files provided as input arguments

  for (int arg = 1; arg < argc; arg++)
  {
    Bundle bundle;
    if (bundle.parseBundle(argv[arg]))
      converter.addBundle(&bundle);
  }

  ros::Rate loop_rate(converter.getUpdateRate());
  while (ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return EXIT_SUCCESS;
}

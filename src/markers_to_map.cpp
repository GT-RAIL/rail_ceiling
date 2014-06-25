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

  // create the ROS topics
  markers_in = node.subscribe<ar_track_alvar::AlvarMarkers>("ar_pose_marker", 1, &markers_to_map::markers_cback, this);
  map_in = node.subscribe<nav_msgs::OccupancyGrid>("map", 1, &markers_to_map::map_in_cback, this);
  map_out = node.advertise<nav_msgs::OccupancyGrid>("marker_map", 1);

  ROS_INFO("Markers To Map Started");
}

float markers_to_map::round(float f,float prec) {
    return (float) (floor(f*(1.0f/prec) + 0.5)/(1.0f/prec));
}

float markers_to_map::min(float a, float b) {
  if (a <= b) return a;
  return b;
}

float markers_to_map::max(float a, float b) {
  if (a >= b) return a;
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
  if (mapReceived) {
    //Initialize map object
    nav_msgs::OccupancyGrid map;
    map.header.frame_id = "ar_map";
    map.header.stamp = ros::Time::now();
    map.info = globalMap.info;
    vector<signed char> mapData(map.info.width * map.info.height);
    fill(mapData.begin(), mapData.end(), -1);



    //TODO: Add border based on marker size;

    //Iterate over every marker bundle
    for(int i = 0; i < markers->markers.size(); i++) {

      //TODO: better name for these and related variables
      float xAbsLength = 0;
      float yAbsLength = 0;

      //find the relevant bundle
      for (int j = 0; j < bundles.size(); j++) {
        if (bundles[j]->getId() == markers->markers[i].id) {
          xAbsLength = bundles[j]->getBundleWidth();
          yAbsLength = bundles[j]->getBundleHeight();
        }
      }
      if (xAbsLength == 0 && yAbsLength == 0) {
        ROS_ERROR("AR ID not found in list of bundles");
        continue;
      }

      //TODO: get widths from bundle xml file (convert to m)

      try {
        //Find transform and discretize sizes to grid
        tf::StampedTransform transform;
        listener.lookupTransform("/ar_map", "/ar_marker_"+(boost::lexical_cast<string>(markers->markers[i].id)), ros::Time(0), transform);
        float xAbs = round(transform.getOrigin().x()-map.info.origin.position.x, map.info.resolution);
        float yAbs = round(transform.getOrigin().y()-map.info.origin.position.y, map.info.resolution);
        int xGrid = xAbs/map.info.resolution;
        int yGrid = yAbs/map.info.resolution;
        int xGridLength = round(xAbsLength, map.info.resolution)/map.info.resolution;
        int yGridLength = round(yAbsLength, map.info.resolution)/map.info.resolution;

        //Create the obstacle in its own grid
        nav_msgs::OccupancyGrid obstacle;
        vector<signed char> obstacleData(xGridLength * yGridLength);
        fill(obstacleData.begin(), obstacleData.end(), 127);
        obstacle.info.width=xGridLength;
        obstacle.info.height=yGridLength;

        //Find the needed rotation
        tf::Quaternion q(transform.getRotation().x(), transform.getRotation().y(), transform.getRotation().z(), transform.getRotation().w());
        double roll, pitch, yaw;
        tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
        float angle = yaw;
        float cosine=(float)cos(angle);
        float sine=(float)sin(angle);

        //Calculate dimensions of rotated obstacles bounding box
        float height = (float)obstacle.info.height;
        float width = (float)obstacle.info.width;
        float Point1x=(-height*sine);
        float Point1y=(height*cosine);
        float Point2x=(width*cosine-height*sine);
        float Point2y=(height*cosine+width*sine);
        float Point3x=(width*cosine);
        float Point3y=(width*sine);
        float minx=min(0,min(Point1x,min(Point2x,Point3x)));
        float miny=min(0,min(Point1y,min(Point2y,Point3y)));
        float maxx=max(Point1x,max(Point2x,Point3x));
        float maxy=max(Point1y,max(Point2y,Point3y));
        int DestWidth=(int)ceil(fabs(maxx)-minx);
        int DestHeight=(int)ceil(fabs(maxy)-miny);

        //Calculate offset from bounding box for drawing on map based on the quadrant the obstacle was rotated into
        int xOffset;
        int yOffset;
        if ((angle >= 0 && angle <= (PI/2)+0.001) || (angle >= -2*PI-0.001 && angle < -3*PI/2)) {
          xOffset = -Point3x+Point1x;
          yOffset = -Point3y;
        } else if ((angle > PI/2 && angle <= PI+0.001) || (angle >= -3*PI/2-0.001 && angle < -PI)) {
          xOffset = Point1x;
          yOffset = -Point3y+Point1y;
        } else if ((angle > PI && angle <= (3*PI/2)+0.001) || (angle >= -PI-0.001 && angle < -PI/2)) {
          xOffset = 0;
          yOffset = Point1y;
        } else if ((angle > (3*PI/2) && angle <= (2*PI)+0.001) || (angle >= -PI/2-0.001 && angle < 0)) {
          xOffset = -Point3x;
          yOffset = 0;
        }

        //Rotate every point on the obstacle and draw it on the map
        for(int x=0;x<DestWidth;x++)
        {
          for(int y=0; y<DestHeight; y++)
          {
            int SrcX=(int)((x+minx)*cosine+(y+miny)*sine);
            int SrcY=(int)((y+miny)*cosine-(x+minx)*sine);
            if(SrcX >= 0 && SrcX < obstacle.info.width && SrcY >= 0 && SrcY < obstacle.info.height)
            {
              mapData[(xGrid+x+xOffset)+(yGrid+y+yOffset)*map.info.width] = obstacleData[SrcX+SrcY*obstacle.info.width];
            }
          }
        }
      }
      catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
      }
    }
    //publish the map
    map.data = mapData;
    map_out.publish(map);
  }
}


void markers_to_map::addBundle(Bundle* bundle) {
  bundles.push_back(bundle);
}

//-------------------
//TODO Move bundle class method definitions into seperate file
//-------------------


Bundle::Bundle() {}

void Bundle::parseBundle(TiXmlDocument doc)
{

  //TODO: This is messy. Make it cleaner.

  TiXmlHandle hDoc(&doc);
  TiXmlElement* pElem;
  TiXmlHandle hRoot(0);

  pElem=hDoc.FirstChildElement().Element();
  if (!pElem) return;
  int markerCount;
  pElem->QueryIntAttribute("markers",&markerCount);
  //ROS_INFO("%s %d",pElem->Value(),markerCount);

  // save this for later
  hRoot=TiXmlHandle(pElem);

  //first marker
  TiXmlHandle markerRoot =  hRoot.FirstChild("marker");
  TiXmlElement* pMarkersNode = markerRoot.Element();

  //save marker id
  pMarkersNode->QueryIntAttribute("index",&id);
 // ROS_INFO("%s %d",pMarkersNode->Value(), id);

  //save marker size
  TiXmlElement* pCornersNode = markerRoot.FirstChild().Element();
  pCornersNode->QueryFloatAttribute("x",&markerSize);
  markerSize = 2*abs(markerSize);
  //ROS_INFO("%s %f",pCornersNode->Value(), markerSize);

  //go to second marker to get width and height
  float temp1;
  float temp2;
  pMarkersNode = pMarkersNode->NextSiblingElement();
  pCornersNode = pMarkersNode->FirstChildElement();
  pCornersNode->QueryFloatAttribute("x",&temp1);
  pCornersNode = pCornersNode->NextSiblingElement();
  pCornersNode->QueryFloatAttribute("x",&temp2);
  bundleWidth = ((abs(temp1)+abs(temp2))/2)/100;
  //ROS_INFO("%s %f",pCornersNode->Value(), bundleWidth);
  pCornersNode->QueryFloatAttribute("y",&temp1);
  pCornersNode = pCornersNode->NextSiblingElement();
  pCornersNode->QueryFloatAttribute("y",&temp2);
  bundleHeight = ((abs(temp1)+abs(temp2))/2)/100;
  //ROS_INFO("%s %f",pCornersNode->Value(), bundleHeight);

}

int Bundle::getId()
{
  return id;
}

float Bundle::getMarkerSize()
{
  return markerSize;
}

float Bundle::getBundleWidth()
{
  return bundleWidth;
}

float Bundle::getBundleHeight()
{
  return bundleHeight;
}

int main(int argc, char **argv)
{
  // initialize ROS and the node
  ros::init(argc, argv, "markers_to_map");

  // initialize the converter
  markers_to_map converter;

  //TODO loaad multiple xml bundle files
  if (argc > 1) {
    TiXmlDocument doc(argv[1]);
    if (!doc.LoadFile()) {
      ROS_ERROR("Failed to load bundle: %s", argv[1]);
      return EXIT_FAILURE;
    }
    Bundle aBundle;
    aBundle.parseBundle(doc);

    converter.addBundle(&aBundle);
  }



  ros::spin();

  return EXIT_SUCCESS;
}

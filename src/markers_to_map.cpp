//TODO FIX comments

/*!
 * \odom_covariance_converter.cpp
 * \brief Adds covariance matrix to odometry message
 *
 * odom_covariance_converter adds a covariance matrix to odometry messages so they are compatible with robot_pose_efk.
 *
 * \author Steven Kordell, WPI - spkordell@wpi.edu
 * \date June 16, 2014
 */

#include <rail_ceiling/markers_to_map.h>

using namespace std;

markers_to_map::markers_to_map()
{
  //a private handle for this ROS node
  ros::NodeHandle node("~");

  // create the ROS topics
  markers_in = node.subscribe<ar_track_alvar::AlvarMarkers>("ar_pose_marker", 10, &markers_to_map::markers_cback, this);
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

void markers_to_map::markers_cback(const ar_track_alvar::AlvarMarkers::ConstPtr& markers)
{
  //Initialize map object
  nav_msgs::OccupancyGrid map;
  //TODO: finish header
  map.header.frame_id = "ar_map";
  //TODO: parameterize these or get them from the existing map
  map.info.resolution = 0.025;
  map.info.width = 338;
  map.info.height = 427;
  map.info.origin.position.x = -4.25;
  map.info.origin.position.y = -5.25;
  map.info.origin.position.z = 0.0;
  map.info.origin.orientation.x = 0.0;
  map.info.origin.orientation.x = 0.0;
  map.info.origin.orientation.z = 0.0;
  map.info.origin.orientation.w = 1.0;
  vector<signed char> mapData(map.info.width * map.info.height);
  fill(mapData.begin(), mapData.end(), -1);

  float res = map.info.resolution;
  tf::StampedTransform transform;
  float xAbs;
  float yAbs;
  //TODO: get widths from bundle xml file (convert to m)
  float xAbsWidth = 111.5/100;
  float yAbsWidth = 39.5/100;
  int xGrid;
  int yGrid;
  int xGridWidth;
  int yGridWidth;

  for(int i = 0; i != markers->markers.size(); i++) {
    try{
      listener.lookupTransform("/ar_map", "/ar_marker_0",ros::Time(0), transform); //TODO,  generalize ar_marker_0

      xAbs = round(transform.getOrigin().x()-map.info.origin.position.x, map.info.resolution);
      yAbs = round(transform.getOrigin().y()-map.info.origin.position.y, map.info.resolution);
      xGrid = xAbs/res;
      yGrid = yAbs/res;
      xGridWidth = round(xAbsWidth, res)/res;
      yGridWidth = round(yAbsWidth, res)/res;

      nav_msgs::OccupancyGrid obstacle;
      vector<signed char> obstacleData(xGridWidth * yGridWidth);
      fill(obstacleData.begin(), obstacleData.end(), 127);
      obstacle.info.width=xGridWidth;
      obstacle.info.height=yGridWidth;

      //TODO: better way to do this? Probably don't even have to convert it since we're using a bunch of sines cosines anyhow
      tf::Quaternion q(transform.getRotation().x(), transform.getRotation().y(), transform.getRotation().z(), transform.getRotation().w());
      double roll, pitch, yaw;
      tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

      float angle = yaw;
      float radians = angle;

      float cosine=(float)cos(radians);
      float sine=(float)sin(radians);

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

      int DestBitmapWidth=(int)ceil(fabs(maxx)-minx);
      int DestBitmapHeight=(int)ceil(fabs(maxy)-miny);

      int xOffset;
      int yOffset;
      if ((radians >= 0 && radians <= (PI/2)+0.001) || (radians >= -2*PI-0.001 && radians < -3*PI/2)) {
        xOffset = -Point3x+Point1x;
        yOffset = -Point3y;
      } else if ((radians > PI/2 && radians <= PI+0.001) || (radians >= -3*PI/2-0.001 && radians < -PI)) {
        xOffset = Point1x;
        yOffset = -Point3y+Point1y;
      } else if ((radians > PI && radians <= (3*PI/2)+0.001) || (radians >= -PI-0.001 && radians < -PI/2)) {
        xOffset = 0;
        yOffset = Point1y;
      } else if ((radians > (3*PI/2) && radians <= (2*PI)+0.001) || (radians >= -PI/2-0.001 && radians < 0)) {
        xOffset = -Point3x;
        yOffset = 0;
      }
      for(int x=0;x<DestBitmapWidth;x++)
      {
        for(int y=0; y<DestBitmapHeight; y++)
        {
          int SrcBitmapx=(int)((x+minx)*cosine+(y+miny)*sine);
          int SrcBitmapy=(int)((y+miny)*cosine-(x+minx)*sine);
          if(SrcBitmapx >= 0 && SrcBitmapx < obstacle.info.width && SrcBitmapy >= 0 && SrcBitmapy < obstacle.info.height)
          {
            mapData[(xGrid+x+xOffset)+(yGrid+y+yOffset)*map.info.width] = obstacleData[SrcBitmapx+SrcBitmapy*obstacle.info.width];
          }
        }
      }
      map.data = mapData;
      map_out.publish(map);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
    }
  }

  //TODO cite: http://www.leunen.com/cbuilder/rotbmp.html

}



int main(int argc, char **argv)
{
  // initialize ROS and the node
  ros::init(argc, argv, "markers_to_map");

  // initialize the converter
  markers_to_map converter;

  ros::spin();

  return EXIT_SUCCESS;
}

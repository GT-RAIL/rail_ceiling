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


/*
 *
 * TODO: Remove this
 *
 */

/*
void markers_to_map::updateMarkerList(const ar_track_alvar::AlvarMarkers::ConstPtr& marker)
{
  ROS_INFO("%d",marker->);

  //check to see if the marker is already in the list, if so, update that marker
  /*
  bool markerFound = false;
  for(vector<ar_track_alvar::AlvarMarkers::ConstPtr>::iterator it = markerArray.begin(); it != markerArray.end(); it++) {
      if (marker == *it) {
        markerFound = true;
        ROS_INFO("marker found");
      }
  }
  //else, this is a new marker, add it to the list
  if (!markerFound) {
    markerArray.push_back(marker);
    ROS_INFO("marker added");
  }

}
*/
float round(float f,float pres) {
    return (float) (floor(f*(1.0f/pres) + 0.5)/(1.0f/pres));
}

float min(float a, float b) {
  if (a <= b) return a;
  return b;
}

float max(float a, float b) {
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
  /*
  for(int i=0; i<(map.info.width * map.info.height); i++){
    mapData[i] = -1;
  }
  */

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
      listener.lookupTransform("/ar_marker_0", "/ar_map",ros::Time(0), transform); //TODO,  generalize ar_marker_0
      xAbs = round(transform.getOrigin().x()-map.info.origin.position.x, map.info.resolution);
      yAbs = round(transform.getOrigin().y()-map.info.origin.position.y, map.info.resolution);
      xGrid = xAbs/res;
      yGrid = yAbs/res;
      xGridWidth = round(xAbsWidth, res)/res;
      yGridWidth = round(yAbsWidth, res)/res;

      //ROS_INFO("%d  %f   %f",markers->markers[i].id,transform.getOrigin().x(),xTemp);
      //ROS_INFO("%d  %d", xGrid, yGrid);
      ROS_INFO(" %d  %d",xGridWidth, yGridWidth);

      for (int j = 0; j < xGridWidth; j++) {
        for (int k = 0; k < yGridWidth; k++) {
          mapData[(xGrid+j)+(yGrid-k)*map.info.width] = 127;
        }
      }

      //attempting to rotate it
      int angle=45;        //45° for example
      //Convert degrees to radians
      float radians=(2*3.1416*angle)/360;

      float cosine=(float)cos(radians);
      float sine=(float)sin(radians);

      float Point1x=(-(float)map.info.height*sine);
      float Point1y=((float)map.info.height*cosine);
      float Point2x=((float)map.info.width*cosine-(float)map.info.height*sine);
      float Point2y=((float)map.info.height*cosine+(float)map.info.width*sine);
      float Point3x=((float)map.info.width*cosine);
      float Point3y=((float)map.info.width*sine);

      float minx=min(0,min(Point1x,min(Point2x,Point3x)));
      float miny=min(0,min(Point1y,min(Point2y,Point3y)));
      float maxx=max(Point1x,max(Point2x,Point3x));
      float maxy=max(Point1y,max(Point2y,Point3y));

      int DestBitmapWidth=(int)ceil(fabs(maxx)-minx);
      int DestBitmapHeight=(int)ceil(fabs(maxy)-miny);


      vector<signed char> rotateMapData(DestBitmapWidth * DestBitmapHeight);

      for(int x=0;x<DestBitmapWidth;x++)
      {
        for(int y=0;y<DestBitmapHeight;y++)
        {
          int SrcBitmapx=(int)((x+minx)*cosine+(y+miny)*sine);
          int SrcBitmapy=(int)((y+miny)*cosine-(x+minx)*sine);
          if(SrcBitmapx >= 0 && SrcBitmapx < map.info.width && SrcBitmapy >= 0 && SrcBitmapy < map.info.height)
          {
            rotateMapData[x+y*DestBitmapWidth] = mapData[SrcBitmapx+SrcBitmapy*map.info.width];
          }
        }
      }

      map.info.width = DestBitmapWidth;
      map.info.height = DestBitmapHeight;
      map.data = rotateMapData;
      //map.data = mapData;
      map_out.publish(map);

    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
    }
  }



  //cite: http://www.leunen.com/cbuilder/rotbmp.html



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

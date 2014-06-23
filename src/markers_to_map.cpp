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
  for(int i=0; i<(map.info.width * map.info.height); i++){
    mapData[i] = -1;
  }


  float res = map.info.resolution;

  tf::StampedTransform transform;
  //fill map with items
  float xAbs;
  float yAbs;
  int xGrid;
  int yGrid;

  int xGridWidth = 100;
  int yGridWidth = 100;

  for(int i = 0; i != markers->markers.size(); i++) {
    try{
      listener.lookupTransform("/ar_marker_0", "/ar_map",ros::Time(0), transform); //TODO,  generalize ar_marker_0
      xAbs = round(transform.getOrigin().x()-map.info.origin.position.x, map.info.resolution);
      yAbs = round(transform.getOrigin().y()-map.info.origin.position.y, map.info.resolution);
      xGrid = xAbs/res;
      yGrid = yAbs/res;
      //ROS_INFO("%d  %f   %f",markers->markers[i].id,transform.getOrigin().x(),xTemp);

      ROS_INFO("%d  %d", xGrid, yGrid);

      for (int j = 0; j < xGridWidth; j++) {
        for (int k = 0; k < yGridWidth; k++) {
          mapData[(xGrid+j)+(yGrid+k)*map.info.width] = 127;
        }
      }
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
    }
  }



  map.data = mapData;
  map_out.publish(map);

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

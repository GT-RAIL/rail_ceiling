//TODO: DOXYGEN

#ifndef MARKER_CALLBACK_FUNCTOR_H_
#define MARKER_CALLBACK_FUNCTOR_H_

#include <ros/ros.h>
#include <ar_track_alvar/AlvarMarkers.h>
#include <visualization_msgs/Marker.h>

class MarkerCallbackFunctor
{
public:
  MarkerCallbackFunctor(std::vector<ar_track_alvar::AlvarMarkers::ConstPtr>* markerDataIn, int cameraNumber);
  void operator()(const ar_track_alvar::AlvarMarkers::ConstPtr& markers);
private:
  int cameraNumber;
  std::vector<ar_track_alvar::AlvarMarkers::ConstPtr>* markerDataIn;
};

//TODO: move
class MarkerVisCallbackFunctor
{
public:
  MarkerVisCallbackFunctor(std::vector<std::vector<visualization_msgs::Marker::ConstPtr> >* markerVisDataIn, int cameraNumber);
  void operator()(const visualization_msgs::Marker::ConstPtr& vis_marker);
private:
  int cameraNumber;
  std::vector<std::vector<visualization_msgs::Marker::ConstPtr> >* markerVisDataIn;
};

#endif //MARKER_CALLBACK_FUNCTOR_H_

/*!
 * \marker_vis_callback_functor.h
 * \brief functor for storing incoming marker visualization data associated with a camera for a ros callback
 *
 * Creates a functor (function object) used as a ros callback to allow subscription to an arbitrary number of marker visualization topics with each callback associated with its source camera
 *
 * \author Steven Kordell, WPI - spkordell@wpi.edu
 * \date July 10, 2014
 */

#ifndef MARKER_VIS_CALLBACK_FUNCTOR_H_
#define MARKER_VIS_CALLBACK_FUNCTOR_H_

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

class MarkerVisCallbackFunctor
{
public:
  MarkerVisCallbackFunctor(std::vector<std::vector<visualization_msgs::Marker::ConstPtr> >* markerVisDataIn,
                           int cameraNumber);
  void operator()(const visualization_msgs::Marker::ConstPtr& vis_marker);
private:
  int cameraNumber;
  std::vector<std::vector<visualization_msgs::Marker::ConstPtr> >* markerVisDataIn;
};

#endif //MARKER_VIS_CALLBACK_FUNCTOR_H_

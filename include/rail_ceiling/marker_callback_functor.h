/*!
 * \marker_callback_functor.h
 * \brief functor for storing incoming marker data associated with a camera for a ros callback
 *
 * Creates a functor (function object) used as a ros callback to allow subscription to an arbitrary number of marker topics with each callback associated with its source camera
 *
 * \author Steven Kordell, WPI - spkordell@wpi.edu
 * \date July 10, 2014
 */

#ifndef MARKER_CALLBACK_FUNCTOR_H_
#define MARKER_CALLBACK_FUNCTOR_H_

#include <ros/ros.h>
#include <ar_track_alvar/AlvarMarkers.h>

class MarkerCallbackFunctor
{
public:
  MarkerCallbackFunctor(std::vector<ar_track_alvar::AlvarMarkers::ConstPtr>* markerDataIn, int cameraNumber);
  void operator()(const ar_track_alvar::AlvarMarkers::ConstPtr& markers);
private:
  int cameraNumber;
  std::vector<ar_track_alvar::AlvarMarkers::ConstPtr>* markerDataIn;
};
#endif //MARKER_CALLBACK_FUNCTOR_H_

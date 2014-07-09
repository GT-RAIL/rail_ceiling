//TODO: DOXYGEN

/*!
 * \bundle.cpp
 * \brief method definitions for working with bundles
 *
 * A bundle is a group of ar markers attached to an object. Bundles are defined in an xml file and
 * used by ar_track_avlar for robustness against occlusion. The bundle class defined here is used
 * for parsing these xml files to obtain information needed for projecting the bundle onto a map to
 * aid in obstacle avoidance.
 *
 * \author Steven Kordell, WPI - spkordell@wpi.edu
 * \date June 26, 2014
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

//TODO: fix comments

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

#include <rail_ceiling/marker_callback_functor.h>

MarkerCallbackFunctor::MarkerCallbackFunctor(std::vector<ar_track_alvar::AlvarMarkers::ConstPtr>* markerDataIn,
                                             int cameraNumber)
{
  this->markerDataIn = markerDataIn;
  this->cameraNumber = cameraNumber;
}

void MarkerCallbackFunctor::operator()(const ar_track_alvar::AlvarMarkers::ConstPtr& markers)
{
  markerDataIn->at(cameraNumber) = markers;
  ROS_INFO("test: %d", cameraNumber);
}

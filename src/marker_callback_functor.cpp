/*!
 * \marker_callback_functor.cpp
 * \brief functor for storing incoming marker data associated with a camera for a ros callback
 *
 * Creates a functor (function object) used as a ros callback to allow subscription to an arbitrary number of marker topics with each callback associated with its source camera
 *
 * \author Steven Kordell, WPI - spkordell@wpi.edu
 * \date July 10, 2014
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
}

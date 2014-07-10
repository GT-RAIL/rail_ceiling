/*!
 * \marker_vis_callback_functor.cpp
 * \brief functor for storing incoming marker visualization data associated with a camera for a ros callback
 *
 * Creates a functor (function object) used as a ros callback to allow subscription to an arbitrary number of marker visualization topics with each callback associated with its source camera
 *
 * \author Steven Kordell, WPI - spkordell@wpi.edu
 * \date July 10, 2014
 */


#include <rail_ceiling/marker_vis_callback_functor.h>
MarkerVisCallbackFunctor::MarkerVisCallbackFunctor(
    std::vector<std::vector<visualization_msgs::Marker::ConstPtr> >* markerVisDataIn, int cameraNumber)
{
  this->markerVisDataIn = markerVisDataIn;
  this->cameraNumber = cameraNumber;
}

void MarkerVisCallbackFunctor::operator()(const visualization_msgs::Marker::ConstPtr& vis_marker)
{
  bool contains = false;
  unsigned int i;
  for (i = 0; i < markerVisDataIn->at(cameraNumber).size(); i++)
  {
    if (markerVisDataIn->at(cameraNumber).at(i)->id == vis_marker->id)
    {
      contains = true;
      break;
    }
  }
  if (contains)
  {
    markerVisDataIn->at(cameraNumber).at(i) = vis_marker;
  }
  else
  {
    markerVisDataIn->at(cameraNumber).push_back(vis_marker);
  }
}

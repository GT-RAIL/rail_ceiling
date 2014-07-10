//TODO: DOXYGEN

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
  //ROS_INFO("test: %d", cameraNumber);
}

//TODO: move below to separate file

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

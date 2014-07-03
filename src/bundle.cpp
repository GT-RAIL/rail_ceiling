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

#include <rail_ceiling/bundle.h>

using namespace std;

Bundle::Bundle()
{
}

bool Bundle::parseBundleFootprint(char* filepath)
{
  TiXmlDocument doc(filepath);
  if (!doc.LoadFile())
  {
    ROS_ERROR("Failed to load bundle footprint from %s", filepath);
    return false;
  }
  TiXmlHandle hDoc(&doc);
  TiXmlElement* pElem;
  TiXmlHandle hRoot(0);

  //get the first element of the document
  pElem = hDoc.FirstChildElement().Element();
  if (!pElem)
    return false;
  // save the root
  hRoot = TiXmlHandle(pElem);

  float temp;
  bool masterMarkerFound = false;
  //go to first footprint element
  TiXmlElement* pFootprintNode = hRoot.FirstChild("footprint").FirstChild().Element();
  for (pFootprintNode; pFootprintNode; pFootprintNode = pFootprintNode->NextSiblingElement())
  {
    ROS_INFO("%s", pFootprintNode->Value());
    footprint.header.frame_id = "map";
    if (boost::iequals(pFootprintNode->Value(), "point"))
    {
      geometry_msgs::Point32* point = new geometry_msgs::Point32();
      pFootprintNode->QueryFloatAttribute("x", &temp);
      point->x = temp;
      pFootprintNode->QueryFloatAttribute("y", &temp);
      point->y = temp;
      point->z = 0;
      footprint.polygon.points.push_back(*point);
    }
    if (!masterMarkerFound && boost::iequals(pFootprintNode->Value(), "marker"))
    {
      masterMarkerFound == true;
      pFootprintNode->QueryIntAttribute("index", &id);
      pFootprintNode->QueryFloatAttribute("x", &markerX);
      pFootprintNode->QueryFloatAttribute("y", &markerY);
      pFootprintNode->QueryFloatAttribute("yaw", &markerYaw);
    }
  }
  ROS_INFO("Loaded bundle from %s with ar_id=%d", filepath, id);

  return true;
}

geometry_msgs::PolygonStamped Bundle::getFootprint()
{
  return footprint;
}

int Bundle::getId()
{
  return id;
}

float Bundle::getMarkerX()
{
  return markerX;
}

float Bundle::getMarkerY()
{
  return markerY;
}

float Bundle::getMarkerYaw()
{
  return markerYaw;
}

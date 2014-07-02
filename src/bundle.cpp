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

bool Bundle::parseBundle(char* filepath)
{

  //load the xml document
  TiXmlDocument doc(filepath);
  if (!doc.LoadFile())
  {
    ROS_ERROR("Failed to load bundle from %s", filepath);
    return false;
  }
  TiXmlHandle hDoc(&doc);
  TiXmlElement* pElem;
  TiXmlHandle hRoot(0);

  //get the first xml element in the document
  pElem = hDoc.FirstChildElement().Element();
  if (!pElem)
    return false;
  int markerCount;
  pElem->QueryIntAttribute("markers", &markerCount);
  //ROS_INFO("%s %d",pElem->Value(),markerCount);

  // save the root for later
  hRoot = TiXmlHandle(pElem);

  //find first marker
  TiXmlHandle markerRoot = hRoot.FirstChild("marker");
  TiXmlElement* pMarkersNode = markerRoot.Element();

  //save marker id
  pMarkersNode->QueryIntAttribute("index", &id);
  //ROS_INFO("first %s id: %d",pMarkersNode->Value(), id);

  //save marker size
  TiXmlElement* pCornersNode = markerRoot.FirstChild().Element();
  pCornersNode->QueryFloatAttribute("x", &markerSize);
  markerSize = (2 * abs(markerSize)) / 100;
  //ROS_INFO("first %s, marker size: %f",pCornersNode->Value(), markerSize);

  //go to second marker, calculate bundle width and height
  float temp1;
  float temp2;
  pMarkersNode = pMarkersNode->NextSiblingElement();
  //ROS_INFO("second %s id: %d",pMarkersNode->Value(), id);
  pCornersNode = pMarkersNode->FirstChildElement();
  pCornersNode->QueryFloatAttribute("x", &temp1);
  pCornersNode = pCornersNode->NextSiblingElement();
  pCornersNode->QueryFloatAttribute("x", &temp2);
  bundleWidth = ((abs(temp1) + abs(temp2)) / 2) / 100;
  flipX = (temp1 < 0) ? false: true;
  //ROS_INFO("%s %f",pCornersNode->Value(), bundleWidth);
  pCornersNode->QueryFloatAttribute("y", &temp1);
  pCornersNode = pCornersNode->NextSiblingElement();
  pCornersNode->QueryFloatAttribute("y", &temp2);
  bundleHeight = ((abs(temp1) + abs(temp2)) / 2) / 100;
  flipY = (temp1 < 0) ? true: false;
  //ROS_INFO("%s %f",pCornersNode->Value(), bundleHeight);

  ROS_INFO("Loaded bundle from %s {ar_id=%d marker_size=%f bundle_width=%f bundle_height=%f}", filepath, id, markerSize,
           bundleWidth, bundleHeight);

  return true;
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
  TiXmlElement* pFootprintNode=hRoot.FirstChild( "footprint" ).FirstChild().Element();
  for( pFootprintNode; pFootprintNode; pFootprintNode=pFootprintNode->NextSiblingElement())
  {
    ROS_INFO("%s",pFootprintNode->Value());
    if (boost::iequals(pFootprintNode->Value(),"point")) {
      geometry_msgs::Point32* point = new geometry_msgs::Point32();
      pFootprintNode->QueryFloatAttribute("x", &temp);
      point->x = temp;
      pFootprintNode->QueryFloatAttribute("y", &temp);
      point->y = temp;
      point->z = 0;
      footprint.polygon.points.push_back(*point);
      footprint.header.frame_id = "map";
    }
    if (!masterMarkerFound && boost::iequals(pFootprintNode->Value(),"marker")) {
      masterMarkerFound == true;
      pFootprintNode->QueryIntAttribute("index",&id);
      pFootprintNode->QueryFloatAttribute("size", &markerSize);
      markerSize /= 100; //convert to meters
      pFootprintNode->QueryFloatAttribute("x",&markerX);
      pFootprintNode->QueryFloatAttribute("y",&markerY);
      pFootprintNode->QueryFloatAttribute("yaw",&markerYaw);
    }
  }
  for (int i = 0; i < footprint.polygon.points.size(); i++) {
    ROS_INFO("%f, %f",footprint.polygon.points[i].x,footprint.polygon.points[i].y);
  }
}

geometry_msgs::PolygonStamped Bundle::getFootprint() {
  return footprint;
}

int Bundle::getId()
{
  return id;
}

float Bundle::getMarkerSize()
{
  return markerSize;
}

float Bundle::getBundleWidth()
{
  return bundleWidth;
}

float Bundle::getBundleHeight()
{
  return bundleHeight;
}


bool Bundle::getFlipX()
{
  return flipX;
}


bool Bundle::getFlipY()
{
  return flipY;
}

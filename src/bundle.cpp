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

geometry_msgs::PolygonStamped* Bundle::parsePolygon(TiXmlElement* polygonElement) {
  //add points to the polygon
  geometry_msgs::PolygonStamped* polygon = new geometry_msgs::PolygonStamped();
  polygon->header.frame_id = "map";
  TiXmlElement* pointElement = polygonElement->FirstChildElement("point");
  for (pointElement; pointElement; pointElement = pointElement->NextSiblingElement())
  {
    geometry_msgs::Point32* point = new geometry_msgs::Point32();
    pointElement->QueryFloatAttribute("x", &(point->x));
    pointElement->QueryFloatAttribute("y", &(point->y));
    point->z = 0;
    polygon->polygon.points.push_back(*point);
  }
  return polygon;
}

layer_t* Bundle::parseLayer(TiXmlElement* layerElement) {
  layer_t* layer = new layer_t();

  //set layer name and type
  layerElement->QueryStringAttribute("name", &(layer->name));
  string temp;
  layerElement->QueryStringAttribute("maptype", &temp);
  if (boost::iequals(temp, "rolling")) {
    layer->mapType = ROLLING;
  } else if (boost::iequals(temp, "match_size")) {
    layer->mapType = MATCH_SIZE;
  } else if (boost::iequals(temp, "match_data")) {
    layer->mapType = MATCH_DATA;
  } else {
    ROS_ERROR("Invalid map type in bundle xml");
  }

  //parse polygons
  TiXmlElement* polygonElement = layerElement->FirstChildElement("polygon");
  for (polygonElement; polygonElement; polygonElement = polygonElement->NextSiblingElement()) {
    layer->footprint.push_back(parsePolygon(polygonElement));
  }

  return layer;
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
    if (!masterMarkerFound && boost::iequals(pFootprintNode->Value(), "marker"))
    {
      masterMarkerFound == true;
      pFootprintNode->QueryIntAttribute("index", &id);
      pFootprintNode->QueryFloatAttribute("x", &markerX);
      pFootprintNode->QueryFloatAttribute("y", &markerY);
      pFootprintNode->QueryFloatAttribute("yaw", &markerYaw);
    }

    if (boost::iequals(pFootprintNode->Value(), "layer")){
      layers.push_back(parseLayer(pFootprintNode));
    }

  }
  ROS_INFO("Loaded bundle from %s with ar_id=%d", filepath, id);
  return true;
}

geometry_msgs::PolygonStamped Bundle::getFootprint()
{
  return footprint;
}

vector<layer_t*>* Bundle::getLayers() {
  return &layers;
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

//TODO: fix header comment

#include <rail_ceiling/bundle.h>

Bundle::Bundle() {}

using namespace std;

void Bundle::parseBundle(TiXmlDocument doc)
{

  //TODO: This is messy. Make it cleaner.

  TiXmlHandle hDoc(&doc);
  TiXmlElement* pElem;
  TiXmlHandle hRoot(0);

  pElem=hDoc.FirstChildElement().Element();
  if (!pElem) return;
  int markerCount;
  pElem->QueryIntAttribute("markers",&markerCount);
  //ROS_INFO("%s %d",pElem->Value(),markerCount);

  // save this for later
  hRoot=TiXmlHandle(pElem);

  //first marker
  TiXmlHandle markerRoot =  hRoot.FirstChild("marker");
  TiXmlElement* pMarkersNode = markerRoot.Element();

  //save marker id
  pMarkersNode->QueryIntAttribute("index",&id);
  //ROS_INFO("first %s id: %d",pMarkersNode->Value(), id);

  //save marker size
  TiXmlElement* pCornersNode = markerRoot.FirstChild().Element();
  pCornersNode->QueryFloatAttribute("x",&markerSize);
  markerSize = 2*abs(markerSize);
  //ROS_INFO("first %s, marker size: %f",pCornersNode->Value(), markerSize);

  //go to second marker to get width and height
  float temp1;
  float temp2;
  pMarkersNode = pMarkersNode->NextSiblingElement();
  //ROS_INFO("second %s id: %d",pMarkersNode->Value(), id);
  pCornersNode = pMarkersNode->FirstChildElement();
  pCornersNode->QueryFloatAttribute("x",&temp1);
  pCornersNode = pCornersNode->NextSiblingElement();
  pCornersNode->QueryFloatAttribute("x",&temp2);
  bundleWidth = ((abs(temp1)+abs(temp2))/2)/100;
  //ROS_INFO("%s %f",pCornersNode->Value(), bundleWidth);
  pCornersNode->QueryFloatAttribute("y",&temp1);
  pCornersNode = pCornersNode->NextSiblingElement();
  pCornersNode->QueryFloatAttribute("y",&temp2);
  bundleHeight = ((abs(temp1)+abs(temp2))/2)/100;
  //ROS_INFO("%s %f",pCornersNode->Value(), bundleHeight);
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

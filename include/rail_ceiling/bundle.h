
//TODO: header comment


#ifndef BUNDLE_H_
#define BUNDLE_H_

#include <ros/ros.h>
#include <tinyxml.h>

//TODO comment
class Bundle
{
public:
  Bundle();
  void parseBundle(TiXmlDocument doc);
  int getId();
  float getMarkerSize();
  float getBundleWidth();
  float getBundleHeight();

private:
  int id;
  float markerSize;
  float bundleWidth;
  float bundleHeight;
};

#endif //BUNDLE_H

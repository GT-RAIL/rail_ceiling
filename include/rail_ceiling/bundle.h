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

#ifndef BUNDLE_H_
#define BUNDLE_H_

#include <ros/ros.h>
#include <tinyxml.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Point.h>
#include <boost/algorithm/string.hpp>

class Bundle
{
public:
  /*!
   * Creates a Bundle object
   */
  Bundle();

  /*!
   * Attempts to parse an xml bundle file at the given file location
   *\param filepath The location of the xml file
   *\returns True is parse is successful. False is parse fails.
   */
  bool parseBundle(char* filepath);
  //TODO: remove above if below is successfully implemented

  //TODO comment
  bool parseBundleFootprint(char* filepath);

  /*
   * Returns the id of the ar marker associated with this bundle
   * \return The id of the ar marker associated with this bundle
   */
  int getId();

  //TODO: comment
  geometry_msgs::PolygonStamped getFootprint();

  /*
   * Returns the size of the ar markers
   * \returns The size of the ar markers
   */
  float getMarkerSize();

  /*
   * Returns the distance between the origins of the two markers along the x axis
   * \returns The distance between the origins of the two markers along the x axis
   */
  float getBundleWidth();

  /*
   * Returns the distance between the origins of the two markers along the y axis
   * \returns The distance between the origins of the two markers along the y axis
   */
  float getBundleHeight();

  /*
   * Returns true if obstacle needs to be flipped along the x axis, false otherwise
   * \returns True if obstacle needs to be flipped along the x axis, false otherwise
   */
  bool getFlipX();


  /*
   * Returns true if obstacle needs to be flipped along the y axis, false otherwise
   * \returns True if obstacle needs to be flipped along the y axis, false otherwise
   */
  bool getFlipY();

private:
  int id; /*!< associated bundle id */
  float markerSize; /*!< marker size */

  //TODO: probably remove the below four
  float bundleWidth; /*!< distance between the origins of the two markers along the x axis */
  float bundleHeight; /*!< distance between the origins of the two markers along the y axis */
  bool flipX; /*< true if obstacle needs to be flipped along the x axis */
  bool flipY; /*< true if obstacle needs to be flipped along the y axis */


  //TODO comment
  geometry_msgs::PolygonStamped footprint;
  float markerX;
  float markerY;
  float markerYaw;
};

#endif //BUNDLE_H

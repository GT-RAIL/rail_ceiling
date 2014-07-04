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


//TODO: comment
enum map_type_t {ROLLING, MATCH_SIZE, MATCH_DATA};
struct layer_t {
  std::string name; /*! <The name of this layer */
  std::vector<geometry_msgs::PolygonStamped*> footprint; /*! <The footprint of the obstacle bundle to be drawn on this layer of the map */
  map_type_t mapType;
};


class Bundle
{
public:
  /*!
   * Creates a Bundle object
   */
  Bundle();

  /*!
   * Attempts to parse an xml bundle file containing the bundle footprint at the given file location
   *\param filepath The location of the xml file
   *\returns True is parse is successful. False is parse fails.
   */
  bool parseBundleFootprint(char* filepath);



  //TODO: remove
  /*!
   * Returns the the bundle footprint
   *\returns the bundle footprint
   */
  geometry_msgs::PolygonStamped getFootprint();


  /*!
   * Returns a pointer to the list of footprint layers
   *\returns A pointer to the list of footprint layers
   */
  std::vector<layer_t*>* getLayers();

  /*
   * Returns the id of the ar marker associated with this bundle
   * \return The id of the ar marker associated with this bundle
   */
  int getId();

  /*!
   * Returns the x position of the marker origin with respect to the footprint origin
   * \returns The x position of the marker origin with respect to the footprint origin
   */
  float getMarkerX();

  /*!
   * Returns the y position of the marker origin with respect to the footprint origin
   * \returns The y position of the marker origin with respect to the footprint origin
   */
  float getMarkerY();

  /*!
   * Returns the rotation of the marker with respect to it's own origin
   * \returns The rotation of the marker with respect to it's own origin
   */
  float getMarkerYaw();

private:

  /*
   * TODO: comment
   */
  layer_t* parseLayer(TiXmlElement* layerElement);
  geometry_msgs::PolygonStamped* parsePolygon(TiXmlElement* polygonElement);

  int id; /*!< associated bundle id */

  std::vector<layer_t*> layers;

  //TODO: remove
  geometry_msgs::PolygonStamped footprint; /*! <The footprint of the obstacle bundle to be drawn on the map */


  float markerX; /*! <The x position of the marker origin with respect to the footprint origin */
  float markerY; /*! <The y position of the marker origin with respect to the footprint origin  */
  float markerYaw; /*! <The rotation of the marker with respect to it's own origin in radians*/
};

#endif //BUNDLE_H

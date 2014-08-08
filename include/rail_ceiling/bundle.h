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
#include <boost/algorithm/string.hpp>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Point.h>
#include <tinyxml.h>

enum map_type_t
{
  ROLLING, MATCH_SIZE, MATCH_DATA
};
struct layer_t
{
  std::string name; /*! <The name of this layer */
  std::vector<geometry_msgs::PolygonStamped*> footprint; /*! <The footprint of the obstacle bundle to be drawn on this layer of the map */
  map_type_t mapType; /*! < The type of this map */
  bool fillPolygon;
};

class Bundle
{
public:
  /*!
   * Creates a Bundle object
   */
  Bundle();

  /*!
   * Frees memory used by bundle object
   */
  ~Bundle();

  /*!
   * Attempts to parse an xml bundle file containing the bundle footprint at the 
   * given file location
   *\param filepath The location of the xml file
   *\returns True is parse is successful. False is parse fails.
   */
  bool parseBundleFootprint(char* filepath);

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

  /*!
   * Returns true if the bundle should be kept on the map if it was completely occluded. Returns false otherwise.
   * \returns True if the bundle should be kept on the map if it was completely occluded. False otherwise.
   */
  bool getKeepOnOcclusion();

private:
  layer_t* parseLayer(TiXmlElement* layerElement); /*! <Parses a footprint layer section of a bundle xml file */
  geometry_msgs::PolygonStamped* parsePolygon(TiXmlElement* polygonElement); /*! <Parses a polygon section of a bundle xml file */

  int id; /*!< associated bundle id */
  std::vector<layer_t*> layers; /*!< All the layers associated with this bundle */
  float markerX; /*! <The x position of the marker origin with respect to the footprint origin */
  float markerY; /*! <The y position of the marker origin with respect to the footprint origin  */
  float markerYaw; /*! <The rotation of the marker with respect to it's own origin in radians */
  bool keepOnOcclusion; /*! <Should this marker be drawn on the map if it was completely occluded? */
};

#endif //BUNDLE_H

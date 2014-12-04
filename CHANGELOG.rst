^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rail_ceiling
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.3 (2014-12-04)
------------------
* old YAML syntax fix 2
* syntax fix for old YAML versions
* Merge branch 'develop' of github.com:WPI-RAIL/rail_ceiling into develop
* Added support for previous YAML versions, removed unused bundles
* Update .travis.yml
* Merge branch 'develop' of github.com:WPI-RAIL/rail_ceiling into develop
* corrected the ratio for the slerp averaging; it was originally backwards
* Contributors: David Kent, Russell Toris

0.0.2 (2014-12-02)
------------------
* Linear interpolation-based method to average quaternions for camera calibration samples
* topic name update
* removed some debugging statements
* CMakeLists install
* Furniture position adjustment
* Organization/Documentation
* Initial positions of furniture can be included in the markers.yaml file, which can be optionally read in when launching the furniture_tracker node
* Added missing dependency
* Documentation and cleanup
* New implementation of furniture tracking for both localization and path planning
* Merge branch 'develop' of https://github.com/WPI-RAIL/rail_ceiling into develop
* Started implementing new furniture tracker with YAML config files
* Update .travis.yml
* Contributors: David Kent, Russell Toris

0.0.1 (2014-08-27)
------------------
* ceiling launch file added
* calibration launch file created
* travis fix
* launch files added for URDF
* URDF files added
* calibration file now written as a URDF
* rviz config added
* travis fix
* calibration of camera 2
* fix for travis build
* calibration node now calculates offsets
* started calibration node
* markers added
* added camera calibration files
* cameras now set exposure via usb_cam
* camera launch files created
* Merge pull request #4 from Spkordell/develop
  Added ability to transmit map on service call
* Fixed merge conflicts
* Cleaning
* When loading a map from a file, will publish metadata after loading
* Removed legs from tables
* Added ability to transmit map on service call
* Added rviz configuration
* Can directly load static map rather than reading it from a topic.
* Added option to stop publishing maps while driving
* Fixed transforms
* Added more cameras to launch
* Fixed transform from map to ceiling
* cleanup
* Fixed includes to use new ar_track_alvar_msgs package
* Merge pull request #1 from Spkordell/develop
  Created Rail Ceiling Package
* Camera transform adjustements
* Added bundle for black table
* Fixed camera frames.
* Update camera transforms
* Added support for unfilled polygons
* Merge branch 'develop' of https://github.com/Spkordell/rail_ceiling into develop
* Localization map will not be published if the robot is navigating
* robot ar tracker fixes
* Bundle adjustements
* Cleaning
* Added xml option to specify whether or not a particular obstacle should be kept on the map when occluded.
* Behavior change. When an object is completely occluded, it's last known location will continue to appear on the map.
* Tagged vertical surfaces
* Merge branch 'develop' of https://github.com/Spkordell/rail_ceiling into develop
* Cleaner startup
* Added launch file for ar_tracking from robot
* Added attribute for copying footprints from existing layers to reduce xml bundle complexity
* Bundle fixes
* bundle fixes
* Moved transform exception try-catch
* Optimizations
* Created urdf for environment
* refactoring
* Fixed memory leak
* Cleanind and doxygen
* Doxygen
* Transforming marker poses to odometry frame for rolling maps
* Added null checks
* Cleaning
* Cleaning
* When multiple cameras see the same marker, now selecting the marker which is closest to the camera.
* Adding markers from multiple cameras to the list of markers
* Can subscribe to multiple ar_marker topics
* ar_track_alvar does not publish transforms to markers in their own namespaces, so switching to using pose data for multicamera support
* Different map types can be published at different rates
* Parameterization
* Merge branch 'dev-layer' of https://github.com/Spkordell/rail_ceiling into dev-layer
* Parameterized map_topic
* Rolling map is published with respect to odometry frame
* Parameter for costmap differentiation
* Added support for rolling maps
* Cleaning
* Brought bundles up to spec
* Footprints can consist of multiple polygons
* static map data added to localization layers
* Multiple footprint layers work
* Publishing a map for each layer
* Publishing a map for each layer
* Parsing bundle layers
* Renamed launch files
* Added bundles
* Fixed issue with obstacle overlap.
* Cleaning
* Cleaning
* Accounting for additional marker yaw
* Optimizations
* Cleaning
* Alignment improvements.
* Change bounding box dimensions
* Change bounding box dimensions
* Can rotate about noncenter point
* Can rotate about noncenter point
* Added obstacle to map
* Merging new and old methodologies
* Fixing alignment issues
* matrix correctly sizes to fit polygon
* Conversion of polygonal footprints to occupancy grid
* Began parsing arbitrary bundle footprint polygons
* chair
* Added chair bundle
* Allignment improvements
* Fixed loop rate interfering with marker id.
* Fixed overwriting problem
* Output map now updates at a specified rate
* Fix for costmap dimensions
* Costmap layer plugin subscribes to marker_map
* Layer plugin test
* Layer plugin test
* Cleaning
* Accounting for marker size
* Commenting
* Added comments.
* Added multiple bundle file support
* Moved bundle class to seperate source file
* Markers to map node using information from bundle xml files to define obstacle dimensions
* Beginning to parse bundle xml
* Cleaning and commenting.
* Published map now matches the parameters of the static map
* Cleaning
* Work on rotation
* Work on rotation
* Work on rotation
* Work on rotation
* Work on rotation
* Work on rotation
* Work on rotation.
* Items are sized properly.
* Began adding items to map
* Added .gitignore
* Began making node to publish map from ar markers.
* Fixed table bundle measurements
* Addded bundle launch file.
* Added launch file for launching webcam and ar_track_alvar
* Added markers
* Created rail_ceiling package.
* initial commit
* Contributors: David Kent, Russell Toris, Steven Kordell, dekent, spkordell

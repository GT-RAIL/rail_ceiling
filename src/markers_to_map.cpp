/*!
 * \markers_to_map.cpp
 * \brief places obstacles on a map at a location corresponding to an ar marker
 *
 * places obstacles on a map at a location corresponding to an ar marker
 *
 * \author Steven Kordell, WPI - spkordell@wpi.edu
 * \date June 25, 2014
 */

#include <rail_ceiling/markers_to_map.h>

using namespace std;

markers_to_map::markers_to_map()
{
  //a private handle for this ROS node
  ros::NodeHandle node("~");

  //initialize variables
  mapReceived = false;

  //Read in the update rate
  node.param<double>("update_rate", updateRate, 0.2);

  // create the ROS topics
  markers_in = node.subscribe < ar_track_alvar::AlvarMarkers
      > ("ar_pose_marker", 1, &markers_to_map::markers_cback, this);
  map_in = node.subscribe < nav_msgs::OccupancyGrid > ("map", 1, &markers_to_map::map_in_cback, this);
  map_out = node.advertise < nav_msgs::OccupancyGrid > ("marker_map", 1);
  footprint_out = node.advertise < geometry_msgs::PolygonStamped > ("bundle_footprint", 1);

  ROS_INFO("Markers To Map Started");
}

float markers_to_map::round(float f, float prec)
{
  return (float)(floor(f * (1.0f / prec) + 0.5) / (1.0f / prec));
}

float markers_to_map::min(float a, float b)
{
  if (a <= b)
    return a;
  return b;
}

float markers_to_map::max(float a, float b)
{
  if (a >= b)
    return a;
  return b;
}

void markers_to_map::map_in_cback(const nav_msgs::OccupancyGrid::ConstPtr& map)
{
  globalMap = *map;
  mapReceived = true;
  ROS_INFO("Map Received");
}

void markers_to_map::markers_cback(const ar_track_alvar::AlvarMarkers::ConstPtr& markers)
{
  if (mapReceived)
  {
    //Initialize map object
    nav_msgs::OccupancyGrid map;
    map.header.frame_id = "ar_map";
    map.header.stamp = ros::Time::now();
    map.info = globalMap.info;
    vector<signed char> mapData(map.info.width * map.info.height);
    //fill(mapData.begin(), mapData.end(), -1);

    //footprint_out.publish(bundles[0]->getFootprint()); //TODO: remove

    //Iterate over every marker bundle
    for (int i = 0; i < markers->markers.size(); i++)
    {
      //find the relevant bundle
      int bundleIndex = -1;
      for (int j = 0; j < bundles.size(); j++)
      {
        if (bundles[j]->getId() == markers->markers[i].id)
        {
          bundleIndex = j;
        }
      }
      if (bundleIndex == -1)
      {
        ROS_WARN("AR ID %d not found in list of bundles", markers->markers[i].id);
        continue;
      }

      try
      {
        //Find transform to ar_marker
        tf::StampedTransform transform;
        listener.lookupTransform("/ar_map", "/ar_marker_" + (boost::lexical_cast < string > (markers->markers[i].id)),
                                 ros::Time(0), transform);
        int xGrid = round(transform.getOrigin().x() - map.info.origin.position.x, map.info.resolution)
            / map.info.resolution;
        int yGrid = round(transform.getOrigin().y() - map.info.origin.position.y, map.info.resolution)
            / map.info.resolution;

        //Find the needed rotation angle
        tf::Quaternion q(transform.getRotation().x(), transform.getRotation().y(), transform.getRotation().z(),
                         transform.getRotation().w());
        double roll, pitch, yaw;
        tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
        float angle = yaw;
        //float angle = PI/4;
        //float angle = yaw * (180 / PI); //Convert to degrees
        //float angle = PI/4;
        //float angle = -20;
        ROS_INFO("%f",angle);



        //todo, need to account for marker yaw
        float rotCenterX = bundles[bundleIndex]->markerX;
        float rotCenterY = bundles[bundleIndex]->markerY;
        //transform the polygonal footprint
        geometry_msgs::PolygonStamped transformedFootprint;
        transformedFootprint.header.frame_id = bundles[bundleIndex]->getFootprint().header.frame_id;
        for (int pt = 0; pt < bundles[bundleIndex]->getFootprint().polygon.points.size(); pt++){
          geometry_msgs::Point32 point = bundles[bundleIndex]->getFootprint().polygon.points[pt];
          //translate by center of rotation
          point.x += rotCenterX;
          point.y += rotCenterY;
          //rotate by desired angle
          float x = point.x*cos(angle)-point.y*sin(angle);
          float y = point.x*sin(angle)+point.y*cos(angle);
          point.x = x;
          point.y = y;
          //translate back to origin
          point.x -= rotCenterX;
          point.y -= rotCenterY;
          transformedFootprint.polygon.points.push_back(point);
        }
        footprint_out.publish(transformedFootprint); //TODO: remove

        //TODO: a bitonal image would probably be better to use
        //TODO: consider caching obstacle images to reduce processing needed
        //find min and max points of the polygon footprint
        float minX = numeric_limits<float>::max();
        float maxX = -numeric_limits<float>::max();
        float minY = numeric_limits<float>::max();
        float maxY = -numeric_limits<float>::max();
        for (int pt = 0; pt < transformedFootprint.polygon.points.size(); pt++){
          float x = transformedFootprint.polygon.points[pt].x;
          float y = transformedFootprint.polygon.points[pt].y;
          if (x < minX) minX = x;
          if (x > maxX) maxX = x;
          if (y < minY) minY = y;
          if (y > maxY) maxY = y;
        }
        maxX = round(maxX,map.info.resolution)/map.info.resolution;
        minX = round(minX,map.info.resolution)/map.info.resolution;
        maxY = round(maxY,map.info.resolution)/map.info.resolution;
        minY = round(minY,map.info.resolution)/map.info.resolution;
        int width = abs(maxX) - minX;
        int height = abs(maxY) - minY;

        /*
        int centerX = round(bundles[0]->markerX,map.info.resolution)/map.info.resolution;
        int centerY = round(bundles[0]->markerX,map.info.resolution)/map.info.resolution;
        centerX += abs(centerX);
        centerY += abs(centerY);
         */

        //int centerX = round(bundles[bundleIndex]->markerX,map.info.resolution)/map.info.resolution + width/2;
        //int centerY = round(bundles[bundleIndex]->markerY,map.info.resolution)/map.info.resolution + height/2;


        //ROS_INFO("%d,%d",centerX,centerY);

        //rasterize polygon footprint
        //ROS_INFO("---");
        cv::Mat obsMat = cv::Mat::zeros(height, width, CV_8UC3 );
        int lineType = 8; //TODO: reconsider line type
        cv::Point obsPoints[transformedFootprint.polygon.points.size()];
        for (int pt = 0; pt < transformedFootprint.polygon.points.size(); pt++){
          int x = round(transformedFootprint.polygon.points[pt].x,map.info.resolution)/map.info.resolution;
          x -= minX;
          int y = round(transformedFootprint.polygon.points[pt].y,map.info.resolution)/map.info.resolution;
          y -= minY;
          obsPoints[pt] = cv::Point(x,y);
         // ROS_INFO("%d,%d",x,y);
        }
        const cv::Point* ppt[1] = { obsPoints };
        int npt[] = { transformedFootprint.polygon.points.size() };
        cv::fillPoly(obsMat, ppt, npt, 1, cv::Scalar( 255, 255, 255 ), lineType);

        //rotate as needed


        /*

        cv::Mat img = cv::imread(filename, 0);

        cv::bitwise_not(img, img);

        std::vector<cv::Point> points;
        cv::Mat_<uchar>::iterator it = img.begin<uchar>();
        cv::Mat_<uchar>::iterator end = img.end<uchar>();
        for (; it != end; ++it)
          if (*it)
            points.push_back(it.pos());

        cv::RotatedRect box = cv::minAreaRect(cv::Mat(points));
        */



        //offset by the center of rotatio

        /*

       cv::Rect brect = cv::RotatedRect(cv::Point2f(centerX,centerY), obsMat.size(), angle).boundingRect(); //center, size, angle
       cv::Point2f pt(centerX, centerY);
       cv::Mat r = cv::getRotationMatrix2D(pt, angle, 1.0);

       double sinv = r.at<double>(0,1);
       double cosv = r.at<double>(0,0);

       r.at<double>(0,2) += brect.size().width/2.0 - centerX;
       r.at<double>(1,2) += brect.size().height/2.0 - centerY;

       cv::Mat dst (brect.size().height, brect.size().width ,obsMat.type());
       cv::warpAffine(obsMat, dst, r, brect.size());

           */
//        int centerX = width/2; //TODO, use location of marker
        //int centerY = height/2;

/*
        int xOff = 0;20;
        int yOff = 0;20;

        cv::Mat dst;
        //cv::Point2f pt(centerX, centerY);
        cv::Point2f pt(0, 0);
        //translate
        cv::Mat r = cv::getRotationMatrix2D(pt, 0, 1.0);
        r.at<double>(0,2) = xOff; //xTranslation
        r.at<double>(1,2) = yOff; //yTranslation
        cv::warpAffine(obsMat, dst, r, cv::Size(width+r.at<double>(0,2),height+r.at<double>(1,2)));
        dst.copyTo(obsMat);

        //rotate
        //cv::Point2f newpt((width+r.at<double>(0,2) /2 ), (height+r.at<double>(1,2)) /2);
        cv::Point2f newpt(0,0);
        cv::Rect brect = cv::RotatedRect(newpt, obsMat.size(), angle).boundingRect(); //center, size, angle
        r = cv::getRotationMatrix2D(newpt, angle, 1.0);
        //double sinv = r.at<double>(0,1);
        //double cosv = r.at<double>(0,0);
        //r.at<double>(0,2) += brect.size().width/2.0 - centerX - xOff;
        //r.at<double>(1,2) += brect.size().height/2.0 - centerY - yOff;
        r.at<double>(0,2) = -xOff;
        r.at<double>(1,2) = -yOff;
        //cv::Mat dst (brect.size().height, brect.size().width ,obsMat.type());
        cv::warpAffine(obsMat, dst, r, brect.size());

*/

        //cv::Mat dst;
/*
        angle = 0;
        cv::Point2f pt(obsMat.rows/2, obsMat.cols/2);
        cv::Rect brect = cv::RotatedRect(pt, obsMat.size(), 0).boundingRect(); //center, size, angle
        cv::Mat r = cv::getRotationMatrix2D(pt, angle, 1.0);
        //double sinv = r.at<double>(0,1);
        //double cosv = r.at<double>(0,0);

        //r.at<double>(0,2) += brect.size().width/2.0 - pt.x;
        //r.at<double>(1,2) += brect.size().height/2.0 - pt.y;

        cv::warpAffine(obsMat, dst, r, brect.size());
*/

        //obsMat.copyTo(dst);

        //convert matrix to occupancy grid //TODO: instead, add directly to map
        nav_msgs::OccupancyGrid obstacle;
        vector<signed char> obstacleData(obsMat.rows * obsMat.cols);
        obstacle.info.width = obsMat.cols;
        obstacle.info.height = obsMat.rows;
        obstacle.info.resolution = map.info.resolution;
        for (int ptX = 0; ptX < obsMat.cols; ptX++) {
          for (int ptY = 0; ptY < obsMat.rows; ptY++){
            cv::Vec3b intensity = obsMat.at<cv::Vec3b>(cv::Point(ptX, ptY));
            uchar blue = intensity.val[0];
            //uchar green = intensity.val[1];
            //uchar red = intensity.val[2];
            obstacleData[ptX+ptY*obsMat.cols] = (blue > 128) ? 100 : 0; //TODO: clean/optimize (only need greyscale images)
          }
        }

        int xOffset = minX + round(rotCenterX,map.info.resolution)/map.info.resolution;
        int yOffset = minY + round(rotCenterY,map.info.resolution)/map.info.resolution;
        //add the obstacle to the map
        for (int x = 0; x < obstacle.info.width; x++) {
          for (int y = 0; y < obstacle.info.height; y++) {
            mapData[(xGrid+x+xOffset)+(yGrid+y+yOffset)*map.info.width] = obstacleData[x+y*obstacle.info.width];
          }
        }
        //todo remove
        //obstacle.data = obstacleData;
        //map_out.publish(obstacle);
      }
      catch (tf::TransformException ex)
      {
        ROS_ERROR("%s", ex.what());
      }
    }

    //TODO: publish the map
    map.data = mapData;
    map_out.publish(map);


/*

    //Iterate over every marker bundle
    for (int i = 0; i < markers->markers.size(); i++)
    {
      float xAbsLength = 0;
      float yAbsLength = 0;
      int xFlipSign;
      int yFlipSign;
      float markerRadius = 0;
      //ROS_INFO("%d,%d", i, markers->markers[i].id);
      //find the relevant bundle
      for (int j = 0; j < bundles.size(); j++)
      {
        if (bundles[j]->getId() == markers->markers[i].id)
        {
          markerRadius = sqrt(2 * pow(bundles[j]->getMarkerSize() / 2, 2));
          xAbsLength = bundles[j]->getBundleWidth() + bundles[j]->getMarkerSize();
          yAbsLength = bundles[j]->getBundleHeight() + bundles[j]->getMarkerSize();
          xFlipSign = bundles[j]->getFlipX() ? -1 : 1;
          yFlipSign = bundles[j]->getFlipY() ? -1 : 1;
          //ROS_INFO("%d %d %d",bundles[j]->getId() , bundles[j]->getFlipX(), bundles[j]->getFlipY());
        }
      }
      if (xAbsLength == 0 || yAbsLength == 0)
      {
        ROS_WARN("AR ID %d not found in list of bundles", markers->markers[i].id);
        continue;
      }

      try
      {
        //Find transform and discretize sizes to grid
        tf::StampedTransform transform;
        listener.lookupTransform("/ar_map", "/ar_marker_" + (boost::lexical_cast < string > (markers->markers[i].id)),
                                 ros::Time(0), transform);
        int xGrid = round(transform.getOrigin().x() - map.info.origin.position.x, map.info.resolution)
            / map.info.resolution;
        int yGrid = round(transform.getOrigin().y() - map.info.origin.position.y, map.info.resolution)
            / map.info.resolution;
        int xGridLength = round(xAbsLength, map.info.resolution) / map.info.resolution;
        int yGridLength = round(yAbsLength, map.info.resolution) / map.info.resolution;

        //Create the obstacle in its own grid
        nav_msgs::OccupancyGrid obstacle;
        vector<signed char> obstacleData(xGridLength * yGridLength);
        fill(obstacleData.begin(), obstacleData.end(), 100);
        obstacle.info.width = xGridLength;
        obstacle.info.height = yGridLength;

        //Find the needed rotation angle
        tf::Quaternion q(transform.getRotation().x(), transform.getRotation().y(), transform.getRotation().z(),
                         transform.getRotation().w());
        double roll, pitch, yaw;
        tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
        float angle = yaw;
        float cosine = (float)cos(angle);
        float sine = (float)sin(angle);

        //Calculate dimensions of rotated obstacles bounding box
        float height = (float)obstacle.info.height;
        float width = (float)obstacle.info.width;
        float Point1x = (-height * sine);
        float Point1y = (height * cosine);
        float Point2x = (width * cosine - height * sine);
        float Point2y = (height * cosine + width * sine);
        float Point3x = (width * cosine);
        float Point3y = (width * sine);
        float minx = min(0, min(Point1x, min(Point2x, Point3x)));
        float miny = min(0, min(Point1y, min(Point2y, Point3y)));
        float maxx = max(Point1x, max(Point2x, Point3x));
        float maxy = max(Point1y, max(Point2y, Point3y));
        int DestWidth = (int)ceil(fabs(maxx) - minx);
        int DestHeight = (int)ceil(fabs(maxy) - miny);
        int markerRadiusGridX = round(markerRadius * sine, map.info.resolution) / map.info.resolution;
        int markerRadiusGridY = round(markerRadius * cosine, map.info.resolution) / map.info.resolution;

        //Calculate offset from bounding box for drawing on map based on the quadrant the obstacle was rotated into
        int xOffset;
        int yOffset;
        if ((angle >= 0 && angle <= (PI / 2) + 0.001) || (angle >= -2 * PI - 0.001 && angle < -3 * PI / 2))
        {
          xOffset = -Point3x + Point1x;
          yOffset = -Point3y;
          //ROS_INFO("case 1");
          markerRadiusGridY *= -1;
        }
        else if ((angle > PI / 2 && angle <= PI + 0.001) || (angle >= -3 * PI / 2 - 0.001 && angle < -PI))
        {
          xOffset = Point1x;
          yOffset = -Point3y + Point1y;
          markerRadiusGridX *= -1;
          markerRadiusGridY *= -1;
          //ROS_INFO("case 2");
        }
        else if ((angle > PI && angle <= (3 * PI / 2) + 0.001) || (angle >= -PI - 0.001 && angle < -PI / 2))
        {
          xOffset = 0;
          yOffset = Point1y;
          markerRadiusGridY *= -1;
          //ROS_INFO("case 3");
        }
        else if ((angle > (3 * PI / 2) && angle <= (2 * PI) + 0.001) || (angle >= -PI / 2 - 0.001 && angle < 0))
        {
          xOffset = -Point3x;
          yOffset = 0;
          markerRadiusGridY *= -1;
          //ROS_INFO("case 4");
        }

       // ROS_INFO("%f, %d, %d", angle, markerRadiusGridX, markerRadiusGridY);

        //Rotate every point on the obstacle and draw it on the map
        for (int x = 0; x < DestWidth; x++)
        {
          for (int y = 0; y < DestHeight; y++)
          {
            int SrcX = (int)((x + minx) * cosine + (y + miny) * sine);
            int SrcY = (int)((y + miny) * cosine - (x + minx) * sine);
            if (SrcX >= 0 && SrcX < obstacle.info.width && SrcY >= 0 && SrcY < obstacle.info.height)
            {
              mapData[(xGrid + x + xOffset + markerRadiusGridX)
                  + (yGrid + y + yOffset + markerRadiusGridY) * map.info.width] = obstacleData[SrcX
                  + SrcY * obstacle.info.width];
            }
          }
        }
      }
      catch (tf::TransformException ex)
      {
        ROS_ERROR("%s", ex.what());
      }
    }
    //publish the map
    map.data = mapData;
    map_out.publish(map);
    */
  }
}

void markers_to_map::addBundle(Bundle* bundle)
{
  bundles.push_back(bundle);
}

Bundle* markers_to_map::getBundle(int index)
{
  return bundles[index];
}


double markers_to_map::getUpdateRate()
{
  return updateRate;
}

int main(int argc, char **argv)
{
  // initialize ROS and the node
  ros::init(argc, argv, "markers_to_map");

  // initialize the converter
  markers_to_map converter;

  ros::Rate loop_rate(converter.getUpdateRate());

  //Parse bundle files provided as input arguments
  for (int arg = 1; arg < argc; arg++)
  {
    Bundle* bundle = new Bundle();
    if (bundle->parseBundle(argv[arg]))
      converter.addBundle(bundle);
    bundle->parseBundleFootprint(argv[arg]);
    converter.footprint_out.publish(bundle->getFootprint());


  }
/*
  converter.footprint_out.publish(converter.getBundle(0)->getFootprint());

  for (int i = 0; i < converter.getBundle(0)->getFootprint().points.size(); i++) {
    ROS_INFO("%f, %f",converter.getBundle(0)->getFootprint().points[i].x,converter.getBundle(0)->getFootprint().points[i].y);
  }
*/

  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return EXIT_SUCCESS;
}

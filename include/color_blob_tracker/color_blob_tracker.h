#ifndef COLOR_BLOB_TRACKER_H_
#define COLOR_BLOB_TRACKER_H_

#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <image_transport/image_transport.h>

class ColorBlobTracker {
public:
  ColorBlobTracker( );
  virtual ~ColorBlobTracker();

  void imageCallback( const sensor_msgs::ImageConstPtr& msg);
  cv::Rect findBoundingRect( cv::Mat image );
  ros::NodeHandle m_nh;
  image_transport::ImageTransport m_it;
  image_transport::Subscriber m_sub;

};

#endif // COLOR_BLOB_TRACKER_H_

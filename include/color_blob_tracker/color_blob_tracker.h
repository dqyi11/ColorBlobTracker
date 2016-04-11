#ifndef COLOR_BLOB_TRACKER_H_
#define COLOR_BLOB_TRACKER_H_

#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <image_transport/image_transport.h>

class ColorBlobTracker {
public:
  ColorBlobTracker( );
  virtual ~ColorBlobTracker();

  void image_callback( const sensor_msgs::ImageConstPtr& msg);
  cv::Rect find_bounding_rect( cv::Mat& image, cv::Mat& hue_img );
  int visualization( cv::Rect& bounding_rect, cv::Mat& img );
  bool is_current_target_reached( int x, int y, int target_x, int target_y );
  static void mouse_click(int event, int x, int y, int flags, void* param); 
 
  ros::NodeHandle m_nh;
  image_transport::ImageTransport m_it;
  image_transport::Subscriber m_sub;
  ros::Publisher m_tracked_pos_pub;
  ros::Publisher m_target_pos_pub;

  std::vector< std::pair<cv::Point, bool> > m_target_pos_list;
  int                                       m_target_pos_idx;

};

#endif // COLOR_BLOB_TRACKER_H_

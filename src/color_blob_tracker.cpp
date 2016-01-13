#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include "color_blob_tracker/color_blob_tracker.h"

#define COLOR_BLOB_TRACKER_VIEW "Color Blob Tracker"

using namespace std;
using namespace cv;

cv::Scalar lower_red_hue_lb(0,100,100);
cv::Scalar lower_red_hue_ub(10,255,255);
cv::Scalar upper_red_hue_lb(160,100,100);
cv::Scalar upper_red_hue_ub(179,255,255);

int morph_size = 3;
cv::Mat element = cv::getStructuringElement( cv::MORPH_ELLIPSE, cv::Size( 4*morph_size + 1, 2*morph_size+1 ), cv::Point( morph_size, morph_size ) ); 

void ColorBlobTracker::imageCallback( const sensor_msgs::ImageConstPtr& msg) {
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy( msg, "bgr8" ); 
  }
  catch( cv_bridge::Exception& e ) {
    ROS_ERROR( "cv_bridge exce[topm: %s", e.what() );
    return;
  }
 

  cv::Rect bounding_rect = findBoundingRect( cv_ptr->image );
  rectangle( cv_ptr->image, bounding_rect, Scalar(0,255,0), 1, 8, 0);
  cv::imshow(COLOR_BLOB_TRACKER_VIEW, cv_ptr->image );
  int key_value = cv::waitKey(30);
  if( key_value == (int)('q') ) {
    ros::shutdown();
  }
}

ColorBlobTracker::ColorBlobTracker( ) : m_it( m_nh )  {
  cv::namedWindow(COLOR_BLOB_TRACKER_VIEW);
  cv::startWindowThread();
  
  m_sub = m_it.subscribe("/usb_cam/image_raw", 1, &ColorBlobTracker::imageCallback, this);
}

ColorBlobTracker::~ColorBlobTracker() {
  cv::destroyWindow( COLOR_BLOB_TRACKER_VIEW );
}
 
cv::Rect ColorBlobTracker::findBoundingRect( cv::Mat image ) {
  cv::Rect bounding_rect;
  cv::Mat hsv_image;
  cv::Mat lower_red_hue_range;
  cv::Mat upper_red_hue_range;
  cv::Mat red_hue_image;

  cv::cvtColor( image, hsv_image, cv::COLOR_BGR2HSV );
  cv::inRange(hsv_image, lower_red_hue_lb, lower_red_hue_ub, lower_red_hue_range );
  cv::inRange(hsv_image, upper_red_hue_lb, upper_red_hue_ub, upper_red_hue_range );
  cv::addWeighted( lower_red_hue_range, 1.0, upper_red_hue_range, 1.0, 0.0, red_hue_image );

  cv::Mat tmp_image1, tmp_image2;
  cv::erode( red_hue_image, tmp_image1, element );
  cv::dilate( tmp_image1, tmp_image2, element );
  cv::dilate( tmp_image2, tmp_image1, element );
  cv::erode( tmp_image1, red_hue_image, element );

  vector< vector<Point> > contours;
  vector<Vec4i> hierarchy;
  int largest_contour_index=0;
  int largest_area=0; 
  findContours( red_hue_image, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE );   
  for( int i = 0; i< contours.size(); i++ )  {
    double a=contourArea( contours[i],false);  
    if(a>largest_area){
      largest_area=a;
      largest_contour_index=i;               
      bounding_rect=boundingRect(contours[i]);
    }
  }
  /*
  Scalar color = Scalar( 0, 255, 0 );
  drawContours( image, contours, largest_contour_index, color, 2, 8, hierarchy, 0, Point() );
  cv::imshow(COLOR_BLOB_TRACKER_VIEW, image );
  cv::waitKey(30);
  */
  return bounding_rect;
}

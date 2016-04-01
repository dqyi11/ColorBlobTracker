#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
//#include <ros/param.h>

#include "color_blob_tracker/color_blob_tracker.h"

#define COLOR_BLOB_TRACKER_VIEW "Color Blob Tracker"

using namespace std;
using namespace cv;

// default is red
cv::Scalar lower_hue_lb(0,100,100);
cv::Scalar lower_hue_ub(10,255,255);
cv::Scalar upper_hue_lb(160,100,100);
cv::Scalar upper_hue_ub(179,255,255);

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

  cv::Mat hue_image;
  cv::Rect bounding_rect = findBoundingRect( cv_ptr->image, hue_image );
  //int key_value = visualization( cv_ptr->image );
  int key_value = visualization( bounding_rect, hue_image );
  if( key_value == (int)('q') ) {
    ros::shutdown();
  }
}

ColorBlobTracker::ColorBlobTracker( ) : m_it( m_nh )  {
  cv::namedWindow(COLOR_BLOB_TRACKER_VIEW);
  cv::startWindowThread();
  
  m_sub = m_it.subscribe("/usb_cam/image_raw", 1, &ColorBlobTracker::imageCallback, this);

  vector<int> l_hue_lb;
  if( m_nh.getParam("/colob_blob_tracker/lower_hue_lb", l_hue_lb) ) {
    if( l_hue_lb.size() == 3 ) {
      lower_hue_lb = cv::Scalar( l_hue_lb[0], l_hue_lb[1], l_hue_lb[2] );
    }
  }

  vector<int> l_hue_ub;
  if( m_nh.getParam("/colob_blob_tracker/lower_hue_ub", l_hue_ub) ) {
    if( l_hue_lb.size() == 3 ) {
      lower_hue_lb = cv::Scalar( l_hue_ub[0], l_hue_ub[1], l_hue_ub[2] );
    }
  }

  vector<int> u_hue_lb;
  if( m_nh.getParam("/colob_blob_tracker/upper_hue_lb", u_hue_lb) ) {
    if( u_hue_lb.size() == 3 ) {
      upper_hue_lb = cv::Scalar( u_hue_lb[0], u_hue_lb[1], u_hue_lb[2] );
    }
  }

  vector<int> u_hue_ub;
  if( m_nh.getParam("/colob_blob_tracker/upper_hue_ub", u_hue_ub) ) {
    if( u_hue_ub.size() == 3 ) {
      upper_hue_ub = cv::Scalar( u_hue_ub[0], u_hue_ub[1], u_hue_ub[2] );
    }
  }

}

ColorBlobTracker::~ColorBlobTracker() {
  cv::destroyWindow( COLOR_BLOB_TRACKER_VIEW );
}
 
cv::Rect ColorBlobTracker::findBoundingRect( cv::Mat& image, cv::Mat& hue_image ) {
  cv::Rect bounding_rect;
  cv::Mat hsv_image;
  cv::Mat lower_hue_range;
  cv::Mat upper_hue_range;
  
  cv::cvtColor( image, hsv_image, cv::COLOR_BGR2HSV );
  cv::inRange(hsv_image, lower_hue_lb, lower_hue_ub, lower_hue_range );
  cv::inRange(hsv_image, upper_hue_lb, upper_hue_ub, upper_hue_range );
  cv::addWeighted( lower_hue_range, 1.0, upper_hue_range, 1.0, 0.0, hue_image );

  cv::Mat tmp_image1, tmp_image2;
  cv::erode( hue_image, tmp_image1, element );
  cv::dilate( tmp_image1, tmp_image2, element );
  cv::dilate( tmp_image2, tmp_image1, element );
  cv::erode( tmp_image1, hue_image, element );

  vector< vector<Point> > contours;
  vector<Vec4i> hierarchy;
  int largest_contour_index=0;
  int largest_area=0; 
  findContours( hue_image, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE );   
  for( int i = 0; i< contours.size(); i++ )  {
    double a=contourArea( contours[i],false);  
    if(a>largest_area){
      largest_area=a;
      largest_contour_index=i;               
      bounding_rect=boundingRect(contours[i]);
    }
  }

  return bounding_rect;
}


int ColorBlobTracker::visualization( cv::Rect& bounding_rect, cv::Mat& img ) {
  rectangle( img, bounding_rect, Scalar(0,255,0), 1, 8, 0);
  circle( img, Point2f(bounding_rect.x + bounding_rect.width/2, 
                                 bounding_rect.y + bounding_rect.height/2),
                         2, Scalar(0,255,0), 4 );
  cv::imshow(COLOR_BLOB_TRACKER_VIEW, img );
  return cv::waitKey(30);
}

#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Pose2D.h>
//#include <ros/param.h>

#include "color_blob_tracker/color_blob_tracker.h"

#define COLOR_BLOB_TRACKER_VIEW "Color Blob Tracker"
#define REACH_THRESHOLD 30


using namespace std;
using namespace cv;

int max_h = 180;
int max_s = 255;
int max_v = 255;


// default is red
int lower_hue_lb_h = 0;
int lower_hue_lb_s = 100;
int lower_hue_lb_v = 100;

int lower_hue_ub_h = 10;
int lower_hue_ub_s = 255;
int lower_hue_ub_v = 255;

int upper_hue_lb_h = 160;
int upper_hue_lb_s = 100;
int upper_hue_lb_v = 100;

int upper_hue_ub_h = 179;
int upper_hue_ub_s = 255;
int upper_hue_ub_v = 255;

int morph_size = 3;
cv::Mat element = cv::getStructuringElement( cv::MORPH_ELLIPSE, cv::Size( 4*morph_size + 1, 2*morph_size+1 ), cv::Point( morph_size, morph_size ) ); 

void ColorBlobTracker::image_callback( const sensor_msgs::ImageConstPtr& msg) {
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy( msg, "bgr8" ); 
  }
  catch( cv_bridge::Exception& e ) {
    ROS_ERROR( "cv_bridge exce[topm: %s", e.what() );
    return;
  }

  cv::Mat hue_image;
  cv::Rect bounding_rect = find_bounding_rect( cv_ptr->image, hue_image );

  if( m_target_pos_list.size() > 0 && m_target_pos_idx >= 0 ) {
    pair< Point, bool> current_target = m_target_pos_list[m_target_pos_idx];
    int current_x = bounding_rect.x + bounding_rect.width/2;                                 
    int current_y = bounding_rect.y + bounding_rect.height/2;
    if( is_current_target_reached( current_x, current_y, current_target.first.x, current_target.first.y ) ) {
      m_target_pos_list[m_target_pos_idx].second = false;
      if( m_target_pos_idx < m_target_pos_list.size()-1 ) {
        m_target_pos_idx ++;
        Point new_target_pos = m_target_pos_list[m_target_pos_idx].first;
        geometry_msgs::Pose2D new_target_pos_msg;
        new_target_pos_msg.x = new_target_pos.x;
        new_target_pos_msg.y = new_target_pos.y;
        m_target_pos_pub.publish(new_target_pos_msg); 
         
      }
      else {
        geometry_msgs::Pose2D new_target_pos_msg;
        new_target_pos_msg.x = -1;
        new_target_pos_msg.y = -1;
        m_target_pos_pub.publish(new_target_pos_msg); 
      }
    }
  }

  geometry_msgs::Pose2D tracked_pos_msg;
  if( bounding_rect.width == 0 || bounding_rect.height == 0 ) {
    tracked_pos_msg.x = -1;
    tracked_pos_msg.y = -1;
  }
  else {
    tracked_pos_msg.x = bounding_rect.x + bounding_rect.width/2;
    tracked_pos_msg.y = bounding_rect.y + bounding_rect.height/2;
  } 
  m_tracked_pos_pub.publish(tracked_pos_msg); 

  //int key_value = visualization( cv_ptr->image );
  int key_value = visualization( bounding_rect, cv_ptr->image );
  if( key_value == (int)('q') ) {
    ros::shutdown();
  }
}

ColorBlobTracker::ColorBlobTracker( ) : m_it( m_nh )  {
  
  m_sub = m_it.subscribe("/usb_cam/image_raw", 1, &ColorBlobTracker::image_callback, this);
  m_target_pos_idx = 0;

  vector<int> l_hue_lb;
  if( m_nh.getParam("/colob_blob_tracker/lower_hue_lb", l_hue_lb) ) {
    if( l_hue_lb.size() == 3 ) {
      lower_hue_lb_h = l_hue_lb[0];
      lower_hue_lb_s = l_hue_lb[1];
      lower_hue_lb_v = l_hue_lb[2];
    }
  }

  vector<int> l_hue_ub;
  if( m_nh.getParam("/colob_blob_tracker/lower_hue_ub", l_hue_ub) ) {
    if( l_hue_lb.size() == 3 ) {
      lower_hue_ub_h = l_hue_ub[0];
      lower_hue_ub_s = l_hue_ub[1];
      lower_hue_ub_v = l_hue_ub[2];
    }
  }

  vector<int> u_hue_lb;
  if( m_nh.getParam("/colob_blob_tracker/upper_hue_lb", u_hue_lb) ) {
    if( u_hue_lb.size() == 3 ) {
      upper_hue_lb_h = u_hue_lb[0];
      upper_hue_lb_s = u_hue_lb[1];
      upper_hue_lb_v = u_hue_lb[2];
    }
  }

  vector<int> u_hue_ub;
  if( m_nh.getParam("/colob_blob_tracker/upper_hue_ub", u_hue_ub) ) {
    if( u_hue_ub.size() == 3 ) {
      upper_hue_ub_h = u_hue_ub[0];
      upper_hue_ub_s = u_hue_ub[1];
      upper_hue_ub_v = u_hue_ub[2];
    }
  }

  
  cv::namedWindow(COLOR_BLOB_TRACKER_VIEW);
  cv::setMouseCallback(COLOR_BLOB_TRACKER_VIEW, mouse_click, this );
  /*
  cv::createTrackbar("Lower H lb", COLOR_BLOB_TRACKER_VIEW, &lower_hue_lb_h, max_h);
  cv::createTrackbar("Lower S lb", COLOR_BLOB_TRACKER_VIEW, &lower_hue_lb_s, max_s);
  cv::createTrackbar("Lower V lb", COLOR_BLOB_TRACKER_VIEW, &lower_hue_lb_v, max_v);

  cv::createTrackbar("Lower H ub", COLOR_BLOB_TRACKER_VIEW, &lower_hue_ub_h, max_h);
  cv::createTrackbar("Lower S ub", COLOR_BLOB_TRACKER_VIEW, &lower_hue_ub_s, max_s);
  cv::createTrackbar("Lower V ub", COLOR_BLOB_TRACKER_VIEW, &lower_hue_ub_v, max_v);
  
  cv::createTrackbar("Upper H lb", COLOR_BLOB_TRACKER_VIEW, &upper_hue_lb_h, max_h);
  cv::createTrackbar("Upper S lb", COLOR_BLOB_TRACKER_VIEW, &upper_hue_lb_s, max_s);
  cv::createTrackbar("Upper V lb", COLOR_BLOB_TRACKER_VIEW, &upper_hue_lb_v, max_v);
  
  cv::createTrackbar("Upper H ub", COLOR_BLOB_TRACKER_VIEW, &upper_hue_ub_h, max_h);
  cv::createTrackbar("Upper S ub", COLOR_BLOB_TRACKER_VIEW, &upper_hue_ub_s, max_s);
  cv::createTrackbar("Upper V ub", COLOR_BLOB_TRACKER_VIEW, &upper_hue_ub_v, max_v);
  */

  m_tracked_pos_pub = m_nh.advertise<geometry_msgs::Pose2D>("tracked_pos", 10);
  m_target_pos_pub = m_nh.advertise<geometry_msgs::Pose2D>("target_pos", 10); 

  cv::startWindowThread();
}

ColorBlobTracker::~ColorBlobTracker() {
  cv::destroyWindow( COLOR_BLOB_TRACKER_VIEW );
}
 
cv::Rect ColorBlobTracker::find_bounding_rect( cv::Mat& image, cv::Mat& hue_image ) {
  cv::Rect bounding_rect;
  cv::Mat hsv_image;
  cv::Mat lower_hue_range;
  cv::Mat upper_hue_range;

  cv::Scalar lower_hue_lb(lower_hue_lb_h, lower_hue_lb_s, lower_hue_lb_v);
  cv::Scalar lower_hue_ub(lower_hue_ub_h, lower_hue_ub_s, lower_hue_ub_v);
  cv::Scalar upper_hue_lb(upper_hue_lb_h, upper_hue_lb_s, upper_hue_lb_v);
  cv::Scalar upper_hue_ub(upper_hue_ub_h, upper_hue_ub_s, upper_hue_ub_v);
  
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


  for( unsigned int i=0; i<m_target_pos_list.size(); i++ ) {
    pair< Point, bool > target = m_target_pos_list[i];
    if( true == target.second ) {  
      circle( img, Point2f( target.first.x, target.first.y ), REACH_THRESHOLD, Scalar(255,0,0), 2 ); 
    } 
    else {
      circle( img, Point2f( target.first.x, target.first.y ), 2, Scalar(255,0,0, 0.4), 2 ); 
    }
  }

  cv::imshow(COLOR_BLOB_TRACKER_VIEW, img );
  return cv::waitKey(30);
}

bool ColorBlobTracker::is_current_target_reached( int x, int y, int target_x, int target_y ) {
  double distance = 0.0;
  distance = sqrt( (x-target_x)*(x-target_x) + (y-target_y)*(y-target_y) );
  if( distance < REACH_THRESHOLD ) {
    return true;
  }
  return false;
}  

void ColorBlobTracker::mouse_click(int event, int x, int y, int flags, void* param) {

  ColorBlobTracker* p_object_tracker = static_cast<ColorBlobTracker*>( param );
  if( EVENT_LBUTTONDOWN == event ) {
   
    if( p_object_tracker ) {
      Point pos;
      pos.x = x;
      pos.y = y;
      p_object_tracker->m_target_pos_list.push_back( make_pair( pos, true ) );
    }
  }
  else if( EVENT_RBUTTONDOWN == event ) {
    if( p_object_tracker ) {
      p_object_tracker->m_target_pos_idx = 0;
      if(p_object_tracker->m_target_pos_list.size()>0) {
        pair< Point, bool> first_pos = p_object_tracker->m_target_pos_list[0];
        Point pos = first_pos.first;
        geometry_msgs::Pose2D target_pos_msg;
        target_pos_msg.x = pos.x;
        target_pos_msg.y = pos.y;
        p_object_tracker->m_target_pos_pub.publish(target_pos_msg); 
      }
    }
  } 
  else if( EVENT_MBUTTONDOWN == event ) {
    if( p_object_tracker ) {
      p_object_tracker->m_target_pos_list.clear();
    }
  }
}

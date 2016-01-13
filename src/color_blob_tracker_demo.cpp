#include <ros/ros.h>
#include "color_blob_tracker/color_blob_tracker.h"

int main( int argc, char** argv ) {
  ros::init( argc, argv, "color_blob_tracker" );
  ColorBlobTracker tracker;  
  ros::spin();
  return 0;
}

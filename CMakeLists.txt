# Declare the version of the CMake API for forward-compatibility
cmake_minimum_required(VERSION 2.8)

# Declare the name of the CMake Project
project(colorblob_tracker)

# Find Catkin
find_package(catkin REQUIRED COMPONENTS
             roscpp
             sensor_msgs
             cv_bridge
             image_transport
            )

find_package(OpenCV 2.4 REQUIRED COMPONENTS highgui imgproc  )

# Declare this project as a catkin package
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES hello_world
#  CATKIN_DEPENDS roscpp
  )

# Add the headers from roscpp
include_directories(include ${catkin_INCLUDE_DIRS}
                            ${OpenCV_INCLUDE_DIRS}
                            )

# Define a library target called hello_world
#add_library(hello_world src/hello_world.cpp)
#target_link_libraries(hello_world ${roscpp_LIBRARIES})

add_executable(color_blob_tracker src/color_blob_tracker.cpp
                                  src/color_blob_tracker_demo.cpp
              )
target_link_libraries(color_blob_tracker ${catkin_LIBRARIES} 
                                         ${OpenCV_LIBRARIES}
                     )


cmake_minimum_required(VERSION 3.0.2)
project(robro_image_saver)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  cv_bridge
  roslib
)

catkin_package()

find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(
    /usr/local/lib
    /usr/lib
  )

include_directories(
  ${catkin_INCLUDE_DIRS}
  )

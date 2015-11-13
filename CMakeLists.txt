cmake_minimum_required(VERSION 2.8.3)

project(crops_manipulator_vision)

find_package(catkin REQUIRED COMPONENTS
  tf
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs)

catkin_package()

## Specify additional locations of header files
## Your package locations should be listed before other locations
set(MY_INCLUDES
  include/
)

include_directories(
  ${MY_INCLUDES}
  ${catkin_INCLUDE_DIRS}
)

# Declare the executable
add_executable(tf_cam_broadcaster src/TFBroadcaster.cpp)
add_executable(tf_cam_listener src/TFListener.cpp)

# Specify libraries to link a lib or exec. target against
target_link_libraries(tf_cam_broadcaster ${catkin_LIBRARIES})
target_link_libraries(tf_cam_listener ${catkin_LIBRARIES})

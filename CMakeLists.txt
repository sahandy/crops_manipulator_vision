cmake_minimum_required(VERSION 2.8.3)

project(crops_manipulator_vision)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  pcl_ros
  pcl_conversions
  tf
  cmake_modules
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
add_executable(transform_cloud_to_world src/PointCloudTransformer.cpp)
add_executable(crop_workspace src/CropWorkspace.cpp)

# Specify libraries to link a lib or exec. target against
target_link_libraries(tf_cam_broadcaster ${catkin_LIBRARIES})
target_link_libraries(transform_cloud_to_world ${catkin_LIBRARIES})
target_link_libraries(crop_workspace ${catkin_LIBRARIES})

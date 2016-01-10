cmake_minimum_required(VERSION 2.8.3)

project(crops_manipulator_vision)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  crops_manipulator_gui
  pcl_ros
  pcl_conversions
  tf
  cmake_modules
  sensor_msgs
  cv_bridge
  image_transport)

find_package(Qt4 REQUIRED COMPONENTS QtCore QtGui)

#set definitions for Qt
ADD_DEFINITIONS(${QT_DEFINITIONS})
ADD_DEFINITIONS(-DQT_WEBKIT)
ADD_DEFINITIONS(-DQT_NO_DEBUG)
ADD_DEFINITIONS(-DQT_SHARED)
ADD_DEFINITIONS(-DQT_GUI_LIB)
ADD_DEFINITIONS(-DQT_NETWORK_LIB)
ADD_DEFINITIONS(-DQT_CORE_LIB)

#specify the include dirs
set(QT_MY_INCLUDES
  ${QT_INCLUDE_DIR}
  include)

#qt
#define qt sources
set(QT_SOURCES
  src/VisionInterface.cpp
  src/interface.cpp)

#define qt headers
set(QT_HEADERS
  include/VisionInterface.h)

# define moc headers
# The moc tool reads a C++ header file. If it finds one or more class
# declarations that contain the Q_OBJECT macro, it produces a C++ source file
# containing the meta-object code for those classes.
QT4_WRAP_CPP(QT_MOC_HPP ${QT_HEADERS})
include(${QT_USE_FILE})

## Specify additional locations of header files
## Your package locations should be listed before other locations
# set(MY_INCLUDES
#   include
# )

# now that setting Qt related stuff is done...
# set include directories
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${QT_MY_INCLUDES}
)

# collect my source files
set(INTERFACE_SOURCES
  src/interface.cpp
  ${QT_SOURCES}
  ${QT_MOC_HPP})

set(DETECTOR_SOURCES
  src/FruitDetector.cpp
  src/ClusterExtractor.cpp
  src/Aligner.cpp
)


# catkin_package(
#   CATKIN_DEPENDS roscpp crops_manipulator_gui crops_manipulator_utils pcl_ros pcl_conversions tf cmake_modules sensor_msgs cv_bridge image_transport
#   INCLUDE_DIRS include)

catkin_package()

# Declare the executable
add_executable(tf_cam_broadcaster src/TFBroadcaster.cpp)
add_executable(color_filter_2d src/ColorFilter2D.cpp)
add_executable(transform_cloud_to_world src/PointCloudTransformer.cpp)
add_executable(crop_workspace src/CropWorkspace.cpp)
add_executable(segment src/Segmenter.cpp)
add_executable(color_filter_3d src/ColorFilter3D.cpp)
add_executable(stem_detector src/StemDetector.cpp)
add_executable(clusterizer src/Clusterizer.cpp)
add_executable(align src/AlignFruit.cpp)
add_executable(detector ${DETECTOR_SOURCES})
add_executable(interface ${INTERFACE_SOURCES})

# Specify libraries to link a lib or exec. target against
target_link_libraries(tf_cam_broadcaster ${catkin_LIBRARIES})
target_link_libraries(color_filter_2d ${catkin_LIBRARIES})
target_link_libraries(transform_cloud_to_world ${catkin_LIBRARIES})
target_link_libraries(crop_workspace ${catkin_LIBRARIES})
target_link_libraries(segment ${catkin_LIBRARIES})
target_link_libraries(color_filter_3d ${catkin_LIBRARIES})
target_link_libraries(stem_detector ${catkin_LIBRARIES})
target_link_libraries(clusterizer ${catkin_LIBRARIES})
target_link_libraries(align ${catkin_LIBRARIES})
target_link_libraries(detector ${catkin_LIBRARIES})
target_link_libraries(interface
  ${catkin_LIBRARIES}
  ${QT_LIBRARIES}
  ${QT_GUI_LIB}
  ${QT_CORE_LIB})

#include <ros/ros.h>

#include <iostream>

#include "VisionInterface.h"

int main(int argc, char** argv) {
  VisionInterface *vision_instance = VisionInterface::getInstance();

  std::cout << "Got the VisionInterface instance!" << std::endl;

  ros::spin();

  return 0;
}

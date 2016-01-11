#include <iostream>
#include <sstream>
#include <string>
#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>

#include <ros/ros.h>

#include "FruitDetector.h"

int main(int argc, char** argv) {
  ros::init (argc, argv, "fruit_detector");

  // load the sweet paprika model
  // struct that helps finding the current home directory
  struct passwd *pwd;
  int id = 0;
  if( (pwd = getpwuid(getuid())) == NULL ) {
    ROS_FATAL("Error finding the home directory.");
    return 0;
  }
  std::stringstream ss;
  ss << pwd->pw_dir << "/.crops_manipulator/vision/model/";
  std::cout << "Model located at: " << ss.str() << std::endl;

  std::string full_path;
  std::string file_name = "paprika_centered.pcd";
  full_path.append(ss.str());
  full_path.append(file_name);

  FruitDetector fd(full_path);

  ros::spin();

  return 0;
}

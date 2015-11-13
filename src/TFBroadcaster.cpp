#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>

// Global Variables
std::vector<double> rotation;
std::vector<double> translation;
bool is_rotation_saved = false;
bool is_translation_saved = false;

// helper functions
/**
 * get the rotation values (roll, pitch, yaw) from the line
 */
void getRotation(std::string line) {
  std::stringstream ss(line);
  std::string part;
  while(std::getline(ss, part, ',')) {
    std::stringstream ss(part);
    double value;
    ss >> value;
    rotation.push_back(value);
  }
  is_rotation_saved = true;
}
/**
 * get the translation values (x, y, z) from the line
 */
void getTranslation(std::string line) {
  std::stringstream ss(line);
  std::string part;
  while(std::getline(ss, part, ',')) {
    std::stringstream ss(part);
    double value;
    ss >> value;
    translation.push_back(value);
  }
  is_translation_saved = true;
}

/**
 * Reads the file containing the calibration result and extracts the
 * rotation and translation values.
 */
bool read_tf_from_file() {
  std::ifstream fin;
  fin.open("camera_calibration_result.txt");
  if(!fin.is_open()) {
    std::cout << "could not find the file." << std::endl;
    return false;
  }
  std::string line;
  while(std::getline(fin, line)) {
    if(line.length() == 0) continue;
    if(line[0] == '#') continue;
    if (!is_rotation_saved) {
      getRotation(line);
      continue;
    }
    if (!is_translation_saved) {
      getTranslation(line);
      continue;
    }
  }
  return true;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "cam_tf_broadcaster");
  ros::NodeHandle nh;
  if(!read_tf_from_file())
    return 0;

  std::cout << "rotation: " << std::endl;
  std::cout << rotation[0] << std::endl;
  std::cout << rotation[1] << std::endl;
  std::cout << rotation[2] << std::endl;
  std::cout << "translation: " << std::endl;
  std::cout << translation[0] << std::endl;
  std::cout << translation[1] << std::endl;
  std::cout << translation[2] << std::endl;

  tf::TransformBroadcaster br;
  tf::Transform transform;
  // Set the translation vector
  transform.setOrigin(
    tf::Vector3(translation[0], translation[1], translation[2]));
  tf::Quaternion q;
  // Set the quaternion using fixed axis RPY (roll, pitch, yaw)
  q.setRPY(rotation[0], rotation[1], rotation[2]);
  transform.setRotation(q);
  // rate at which the transformation is published
  ros::Rate rate(10.0);
  while(nh.ok()) {
    /**
     *
      tf::StampedTransform::StampedTransform	(	const tf::Transform & 	input,
      const ros::Time & 	timestamp,
      const std::string & 	parent_frame_id,
      const std::string & 	child_frame_id
      )
     */
    tf::StampedTransform stamped_tf(transform,
                                    ros::Time::now(),
                                    "ideal_cam",
                                    "real_cam");
    // publishing the transformation
    br.sendTransform(stamped_tf);
    rate.sleep();
  }
  return 0;
}

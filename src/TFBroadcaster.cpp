#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#include <fstream>
#include <iostream>
#include <sstream>

// Global Variables
double rotation[4];
double translation[3];

bool read_tf_from_file() {
  std::ifstream fin;
  fin.open("camera_calibration_result.txt");
  if(!fin.is_open()) {
    std::cout << "could not find the file." << std::endl;
    return false;
  }
  std::cout << "found the file" << std::endl;

  std::string line;

  int counter = 0;

  while(std::getline(fin, line)) {
    std::cout<< "reading line: " << line << std::endl;
    if(line.length() == 0) continue;
    std::stringstream ss(line);
    if(counter < 3)
      ss >> rotation[counter];
    else
      ss >> translation[counter-3];
    counter++;
  }

  return true;
}

// void tfCallback(const geometry_msgs::Transform& msg) {
//   static tf::TransformBroadcaster br;
//   tf::Transform transform;
//   tf::Quaternion q(
//                     msg.rotation.x,
//                     msg.rotation.y,
//                     msg.rotation.z,
//                     msg.rotation.w);
//   transform.setOrigin(
//     tf::Vector3(msg.translation.x, msg.translation.y, msg.translation.z));
//   transform.setRotation(q);
//   br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "real_cam", "ideal_cam"));
// }

int main(int argc, char** argv) {
  ros::init(argc, argv, "cam_tf_broadcaster");

  ros::NodeHandle nh;
  ros::Publisher cam_tf_pub_;
  ros::Subscriber cam_tf_sub;

  // cam_tf_pub_ = nh.advertise<geometry_msgs::Transform> ("cam_transformation",1);

  if(!read_tf_from_file())
    return 0;
  geometry_msgs::Transform camTF_msg;
  camTF_msg.translation.x = translation[0];
  camTF_msg.translation.y = translation[1];
  camTF_msg.translation.z = translation[2];
  camTF_msg.rotation.x = rotation[0];
  camTF_msg.rotation.y = rotation[1];
  camTF_msg.rotation.z = rotation[2];
  camTF_msg.rotation.w = 0;

  // cam_tf_pub_.publish(camTF_msg);
  // cam_tf_sub = nh.subscribe("cam_transformation", 10, &tfCallback);

  static tf::TransformBroadcaster br;
  tf::Transform transform;
  tf::Quaternion q(
                    rotation[0],
                    rotation[1],
                    rotation[2],
                    0.0);
  transform.setOrigin(
    tf::Vector3(translation[0], translation[1], translation[2]));
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "real_cam", "ideal_cam"));

  ros::spin();

  return 0;
}

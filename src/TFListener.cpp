#include <iostream>

#include <ros/ros.h>
#include <tf/transform_listener.h>

int main(int argc, char** argv) {

  ros::init(argc, argv, "cam_tf_listener");

  ros::NodeHandle node;
  tf::TransformListener listener;
  ros::Rate rate(10.0);
  while (node.ok()){
    tf::StampedTransform transform;
    try{
      // transform
      // from [first_argument] "/ideal_cam"
      // to [second_argument] "real_cam"
      listener.lookupTransform("/ideal_cam", "/real_cam",
                               ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }

    tf::Quaternion q(transform.getRotation());
    tf::Matrix3x3 rot_m(q);
    double roll, pitch, yaw;
    rot_m.getRPY(roll, pitch, yaw);

    std::cout << "Rotation:" << std::endl;
    std::cout << "    roll = " << roll << std::endl;
    std::cout << "    pitch = " << pitch << std::endl;
    std::cout << "    yaw = " << yaw << std::endl;
    std::cout << "Translation:" << std::endl;
    std::cout << "        X = " << transform.getOrigin().getX() << std::endl;
    std::cout << "        Y = " << transform.getOrigin().getY() << std::endl;
    std::cout << "        Z = " << transform.getOrigin().getZ() << std::endl;
    std::cout << std::endl;

    rate.sleep();
  }
  return 0;
}

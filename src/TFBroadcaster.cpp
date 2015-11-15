#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>

// Global Variables
std::vector<double> r_i_c;
std::vector<double> t_i_c;
double r_w_i[3];
double t_w_i[3];
bool is_ric_saved = false;
bool is_tic_saved = false;

std::ostream& operator<< (std::ostream &out, tf::Transform &transform) {
  double t[3];
  t[0] = transform.getOrigin().getX();
  t[1] = transform.getOrigin().getY();
  t[2] = transform.getOrigin().getZ();

  double r[3];
  tf::Quaternion q(transform.getRotation());
  tf::Matrix3x3 rot_m(q);
  rot_m.getRPY(r[0], r[1], r[2]);

  out << "  Translation:" << std::endl
      << "      X = " << t[0] << std::endl
      << "      Y = " << t[1] << std::endl
      << "      Z = " << t[2] << std::endl
      << "  Rotation:" << std::endl
      << "      roll  = " << r[0] << std::endl
      << "      pitch = " << r[1] << std::endl
      << "      yaw   = " << r[2] << std::endl;

  return out;
}

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
    r_i_c.push_back(value);
  }
  is_ric_saved = true;
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
    t_i_c.push_back(value);
  }
  is_tic_saved = true;
}

/**
 * Reads the file containing the calibration result and extracts the
 * rotation and t_i_c values.
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
    if (!is_ric_saved) {
      getRotation(line);
      continue;
    }
    if (!is_tic_saved) {
      getTranslation(line);
      continue;
    }
  }
  return true;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "cam_tf_broadcaster");
  ros::NodeHandle nh;
  tf::TransformBroadcaster br;
  if(!read_tf_from_file())
    return 0;

  /**
   * A_w_i
   */
  tf::Transform A_w_i;
  A_w_i.setOrigin( tf::Vector3(0.27, 0, 1.574) );
  tf::Quaternion q;
  q.setRPY(-2.0943951023931953, 0, 1.5707963267948966);
  A_w_i.setRotation(q);

  /**
   * A_i_c
   */
  tf::Transform A_i_c;
  // Set the translation vector
  A_i_c.setOrigin(
    tf::Vector3(t_i_c[0], t_i_c[1], t_i_c[2]));
  // Set the quaternion using fixed axis RPY (roll, pitch, yaw)
  q.setRPY(r_i_c[0], r_i_c[1], r_i_c[2]);
  A_i_c.setRotation(q);

  /**
   * A_w_c
   */
  tf::Transform A_w_c = A_i_c * A_w_i;

  tf::Vector3 t_w_c(A_w_c.getOrigin());

  tf::Quaternion q2(A_w_c.getRotation());
  tf::Matrix3x3 r_w_c(q2);

  /**
   * reversing the transformation from the world to camera coordinate system
   * to have what we actually want.
   * r_c_w = transpose ( r_w_c )
   * t_c_w = -r_c_w * t_w_c
   */
  // DO NOT GET CONFUSED!
  // r_c_w --> rotation from camera to world
  // r_w_c --> rotation from world to camera
  // t_c_w --> translation from camera to world
  // t_w_c --> translation from world to camera
  tf::Matrix3x3 r_c_w(r_w_c.transpose());
  tf::Vector3 t_c_w( r_c_w * t_w_c );
  t_c_w = -t_c_w;
  /**
   * Last step: compose the transformation matrix
   * A_c_w
   * transformation from camera coordinate system to world coordinate system
   */
  tf::Transform A_c_w(r_c_w, t_c_w);
  std::cout << "Broadcasting transformation /camera to /world" << std::endl;
  std::cout << A_c_w << std::endl;
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
    tf::StampedTransform stamped_tf(A_c_w,
                                    ros::Time::now(),
                                    "camera",
                                    "/Elem_0");
    // publishing the transformation
    br.sendTransform(stamped_tf);
    rate.sleep();
  }
  return 0;
}

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>
#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>

// Global Variables
std::vector<double> r_i_c_from_file;
std::vector<double> t_i_c_from_file;
bool is_ric_saved = false;
bool is_tic_saved = false;

// helper functions
/**
 * convenience method to output an object of type tf::Matrix3x3
 */
std::ostream& operator<< (std::ostream &out, tf::Matrix3x3 &m) {
  out << "  |  " << m[0].getX() << "\t" << m[0].getY() << "\t" << m[0].getZ() << "  |  " << std::endl
      << "  |  " << m[1].getX() << "\t" << m[1].getY() << "\t" << m[1].getZ() << "  |  " << std::endl
      << "  |  " << m[2].getX() << "\t" << m[2].getY() << "\t" << m[2].getZ() << "  |  " << std::endl;
  return out;
}
/**
 * convenience method to output an object of type tf::Transform
 */
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
      << "      yaw   = " << r[2] << std::endl
      << "  Rotation Matrix:" << std::endl << rot_m << std::endl;

  return out;
}

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
    r_i_c_from_file.push_back(value);
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
    t_i_c_from_file.push_back(value);
  }
  is_tic_saved = true;
}

/**
 * Reads the file containing the calibration result and extracts the
 * rotation and t_i_c values.
 */
bool read_tf_from_file(const std::string& file_path) {
  std::ifstream fin;

  fin.open(file_path.c_str());
  if(!fin.is_open()) {
    ROS_FATAL("Error locating the calibration file. Shutting down the node...");
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

  if(argc > 0) {
    for (int i=0; i<argc; ++i) {
      std::cout << argv[i] << std::endl;
    }
  }
  // Initialize ros node
  ros::init(argc, argv, "cam_tf_broadcaster");
  ros::NodeHandle nh;
  tf::TransformBroadcaster br;

  std::string file_name;
  // reading the parameter for calibration file name
  nh.param<std::string>(
    "/crops_manipulator_vision/calibration_file_name", file_name, "calib.txt");
  std::cout << "reading param file_name: " << file_name << std::endl;

  /**
   * Find the calibration file in home directory
   * path is hard-coded set to ~/.crops_manipulator/vision/
   * file name is received via command line
   */
  // struct that helps finding the current home directory
  struct passwd *pwd;
  int id = 0;
  if( (pwd = getpwuid(getuid())) == NULL ) {
    ROS_FATAL("Error finding the home directory.");
    return 0;
  }
  std::stringstream ss;
  ss << pwd->pw_dir << "/.crops_manipulator/vision/";
  std::cout << "Calibration file located at: " << ss.str() << std::endl;
  std::string full_path;
  full_path.append(ss.str());
  full_path.append(file_name);
  std::cout << "full path: " << full_path << std::endl;

  if(!read_tf_from_file(full_path))
    return 0;

  /**
   * A_w_i
   */
  tf::Matrix3x3 r_w_i;
  // our original angle values:
  // -2.0943951023931953, 0, 1.5707963267948966
  // due to different convention of rotation matrices between AM and ros, we have
  // to multiply the rotation angles by '-1'
  r_w_i.setRPY(2.0943951023931953, 0, -1.5707963267948966);
  tf::Vector3 t_w_i(0.27, 0, 1.574);
  t_w_i = r_w_i * t_w_i;
  t_w_i = -t_w_i;
  tf::Transform A_w_i(r_w_i, t_w_i);
  /**
   * A_i_c
   */
  // Set the translation vector
  tf::Matrix3x3 r_i_c;
  r_i_c.setRPY(-r_i_c_from_file[0], -r_i_c_from_file[1], -r_i_c_from_file[2]);
  tf::Vector3 t_i_c(t_i_c_from_file[0], t_i_c_from_file[1], t_i_c_from_file[2]);
  t_i_c = r_i_c * t_i_c;
  t_i_c = -t_i_c;
  tf::Transform A_i_c(r_i_c, t_i_c);
  /**
   * A_w_c
   */
  tf::Transform A_w_c = A_i_c * A_w_i;

  std::cout << "Publishing the transformation..." << std::endl;
  std::cout << "A_w_c: " << std::endl;
  std::cout << A_w_c << std::endl;

  // rate at which the transformation is published
  ros::Rate rate(30.0); // in 'Hz'
  while(nh.ok()) {
    /**
     *
      tf::StampedTransform::StampedTransform	(	const tf::Transform & 	input,
      const ros::Time & 	timestamp,
      const std::string & 	parent_frame_id,
      const std::string & 	child_frame_id
      )
      the transformation would be from the child frame to the parent frame

     */
    tf::StampedTransform stamped_tf(A_w_c,
                                    ros::Time::now(),
                                    "camera_rgb_optical_frame",
                                    "Elem_0");
    // publishing the transformation
    br.sendTransform(stamped_tf);

    rate.sleep();
  }
  return 0;
}

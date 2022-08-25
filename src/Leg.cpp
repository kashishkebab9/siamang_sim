#include "../include/Leg.h"

Leg::Leg(Eigen::Matrix4d offset_from_torso_, std::string name_) {
  offset_from_torso = offset_from_torso_;
  name = name_;

  l_1 = .5;
  l_2 = .5;
  q0 = 0;
  q1 = 0;

  std::cout << name << " has been constructed with offset: " << std::endl << offset_from_torso << std::endl;
}

void Leg::SetAngles(joint_angles config) {
  std::cout << "Setting the following angles: " << std::endl;
  std::cout << "q0: " << config.q0_ << std::endl;
  std::cout << "q1: " << config.q1_ << std::endl;
  q0 = config.q0_;
  q1 = config.q1_;
  std::cout << "Done!" << std::endl;
}

Leg::position Leg::FwdKinematics() {
  //todo: have this function take w_position as an input
  std::cout << "Calculating Forward Kinematics for " << name << "..." << std::endl;

  Eigen::Vector4d origin_position;
  origin_position << 0, 0, 0, 1;

  //For Joint0 -> Joint1:
  Eigen::Matrix4d t_0_1_rot_z;
  t_0_1_rot_z << cos(q0), -sin(q0), 0, 0,
                 sin(q0),  cos(q0), 0, 0,
                       0,        0, 1, 0,
                       0,        0, 0, 1;

  Eigen::Matrix4d t_0_1_trans;
  t_0_1_trans << 1, 0, 0,   0,
                 0, 1, 0, l_1,
                 0, 0, 1,   0,
                 0, 0, 0,   1;

  Eigen::Matrix4d t_0_1 = t_0_1_rot_z * t_0_1_trans;

  //For Joint1 -> Joint2:
  Eigen::Matrix4d t_1_2_rot_z;
  t_1_2_rot_z << cos(q1), -sin(q1), 0, 0,
                 sin(q1),  cos(q1), 0, 0,
                       0,        0, 1, 0,
                       0,        0, 0, 1;

  Eigen::Matrix4d t_1_2_trans;
  t_1_2_trans << 1, 0, 0,   0,
                 0, 1, 0, l_2,
                 0, 0, 1,   0,
                 0, 0, 0,   1;

  Eigen::Matrix4d t_1_2 = t_0_1_rot_z * t_0_1_trans;
  Eigen::Vector4d vec_output = offset_from_torso * t_0_1 * t_1_2 * origin_position;
  std::cout << "Input Position: "               << std::endl << origin_position << std::endl;
  std::cout << "Transformation Matrix w -> 0: " << std::endl << offset_from_torso << std::endl;
  std::cout << "Transformation Matrix 0 -> 1: " << std::endl << t_0_1 << std::endl;
  std::cout << "Transformation Matrix 1 -> 2: " << std::endl << t_1_2 << std::endl;

  position output;
  output.x = vec_output(0);
  output.y = vec_output(1);
  output.z = vec_output(2);

  std::cout << "Output Position X: " << output.x <<  std::endl;
  std::cout << "Output Position Y: " << output.y <<  std::endl;
  std::cout << "Output Position Z: " << output.z <<  std::endl;

  return output;
}

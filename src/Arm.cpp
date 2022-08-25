#include "../include/Arm.h"


Arm::Arm(Eigen::Matrix4d offset_from_torso_, std::string name_) {
  offset_from_torso = offset_from_torso_;
  name = name_;

  l_1 = 1;
  l_2 = 1;
  l_3 = .5;

  q0 = 0;
  q1 = 0;
  q2 = 0;
  std::cout << name << " has been constructed with offset: " << std::endl << offset_from_torso << std::endl;

}

void Arm::SetAngles(joint_angles config) {
  std::cout << "Setting the following angles: " << std::endl;
  std::cout << "q0: " << config.q0_ << std::endl;
  std::cout << "q1: " << config.q1_ << std::endl;
  std::cout << "q2: " << config.q2_ << std::endl;
  q0 = config.q0_;
  q1 = config.q1_;
  q2 = config.q2_;
  std::cout << "Done!" << std::endl;
}

Arm::position Arm::FwdKinematics() {
  std::cout << "Calculating Forward Kinematics..." << std::endl;

  Eigen::Vector4d origin_position;
  origin_position << 0, 0, 0, 1;
  // For Joint0 -> Joint1:
  Eigen::Matrix4d t_0_1_rot_z;
  t_0_1_rot_z << cos(q0), -sin(q0),   0,     0,
                 sin(q0),  cos(q0),   0,     0,
                       0,        0,   1,     0,
                       0,        0,   0,     1;

  Eigen::Matrix4d t_0_1_trans;
  t_0_1_trans <<  1, 0,   0,  0,
                  0, 1,   0,  l_1,
                  0, 0,   1,  0,
                  0, 0,   0,  1;

  Eigen::Matrix4d t_0_1 = t_0_1_rot_z * t_0_1_trans;

  // For Joint1 -> Joint2:
  Eigen::Matrix4d t_1_2_rot_z;
  t_1_2_rot_z << cos(q1), -sin(q1),   0,     0,
                 sin(q1),  cos(q1),   0,     0,
                       0,        0,   1,     0,
                       0,        0,   0,     1;

  Eigen::Matrix4d t_1_2_trans;
  t_1_2_trans <<  1, 0, 0, 0,
                  0, 1, 0, l_2,
                  0, 0, 1, 0,
                  0, 0, 0, 1;

  Eigen::Matrix4d t_1_2_rot_x;
  t_1_2_rot_x << 1,            0,             0,  0,
                 0,  cos(-M_PI_2),  -sin(-M_PI_2),  0,
                 0,  sin(-M_PI_2),  cos(-M_PI_2),   0,
                 0,            0,            0,   1;

  Eigen::Matrix4d t_1_2 = t_1_2_rot_z * t_1_2_trans * t_1_2_rot_x;

  // For Joint2 -> Joint3:
  Eigen::Matrix4d t_2_3_rot_z;
  t_2_3_rot_z << cos(q2), -sin(q2),   0,     0,
                 sin(q2),  cos(q2),   0,     0,
                       0,        0,   1,     0,
                       0,        0,   0,     1;

  Eigen::Matrix4d t_2_3_trans;
  t_2_3_trans <<  1, 0, 0, 0,
                  0, 1, 0, 0,
                  0, 0, 1, l_3,
                  0, 0, 0, 1;

  Eigen::Matrix4d t_2_3_rot_x;
  t_2_3_rot_x << 1,            0,             0,  0,
                 0,  cos(M_PI_2),  -sin(M_PI_2),  0,
                 0,  sin(M_PI_2),  cos(M_PI_2),   0,
                 0,            0,            0,   1;

  Eigen::Matrix4d t_2_3 = t_2_3_rot_z * t_2_3_trans * t_2_3_rot_x;
  Eigen::Vector4d mat_output = offset_from_torso * t_0_1 * t_1_2 * t_2_3 * origin_position; 


  std::cout <<"Input Position: "               << std::endl << origin_position << std::endl;
  std::cout <<"Transformation Matrix torso -> 0: " << std::endl << offset_from_torso << std::endl;
  std::cout <<"Transformation Matrix 0 -> 1: " << std::endl << t_0_1 << std::endl;
  std::cout <<"Transformation Matrix 1 -> 2: " << std::endl << t_1_2 << std::endl;
  std::cout <<"Transformation Matrix 2 -> 3: " << std::endl << t_2_3 << std::endl;
  position output;
  output.x = mat_output(0);
  output.y = mat_output(1);
  output.z = mat_output(2);
  std::cout <<"Output Position X: " << output.x << std::endl;
  std::cout <<"Output Position Y: " << output.y << std::endl;
  std::cout <<"Output Position Z: " << output.z << std::endl;
  return output;

}

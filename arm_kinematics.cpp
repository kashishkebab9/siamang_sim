#include <iostream>
#include <string>
#include <vector>
#include <Eigen/Dense>
#include <cmath>
#include <math.h>

int main() {
  //inputs:
  double q0 =  0;
  double l_1 = 1;

  double q1 =  0;
  double l_2 = 1;

  double q2 = 0;
  double l_3 = .5;
  
  Eigen::Vector4d w_position;
  w_position << 0, 0, 0, 1;

  // For World -> Joint0:
  Eigen::Matrix4d t_w_0;
  t_w_0 << 1, 0, 0, 0,
           0, 1, 0, 0,
           0, 0, 1, 1,
           0, 0, 0, 1;

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
  

  std::cout <<"Input Position: "               << std::endl << w_position << std::endl;
  std::cout <<"Transformation Matrix w -> 0: " << std::endl << t_w_0 << std::endl;
  std::cout <<"Transformation Matrix 0 -> 1: " << std::endl << t_0_1 << std::endl;
  std::cout <<"Transformation Matrix 1 -> 2: " << std::endl << t_1_2 << std::endl;
  std::cout <<"Transformation Matrix 2 -> 3: " << std::endl << t_2_3 << std::endl;
  std::cout <<"Output Position: "              << std::endl << t_w_0 * t_0_1 * t_1_2 * t_2_3 * w_position << std::endl;


  return 0;
}

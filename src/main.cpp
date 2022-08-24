#include <iostream>
#include <string>
#include "../include/Arm.h"
#include <Eigen/Dense>

int main() {

  Eigen::Matrix4d right_arm_torso_j0;
  right_arm_torso_j0 << 1, 0, 0, 0,
                    0, 1, 0, 0,
                    0, 0, 1, 1,
                    0, 0, 0, 1;

  std::string ra_name = "Right Arm";
  Arm right_arm(right_arm_torso_j0, ra_name);
  Arm::position curr_position = right_arm.FwdKinematics();
  return 0;
}

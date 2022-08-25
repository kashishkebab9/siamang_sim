#include <iostream>
#include <string>
#include "../include/Arm.h"
#include "../include/Leg.h"
#include <Eigen/Dense>

int main() {

  Eigen::Matrix4d right_arm_torso_j0;
  right_arm_torso_j0 << 1, 0, 0, 0,
                    0, 1, 0, 0,
                    0, 0, 1, 1,
                    0, 0, 0, 1;
  Eigen::Matrix4d right_leg_torso_j0;
  right_leg_torso_j0 << 1, 0, 0, 0,
                        0, 1, 0, 3,
                        0, 0, 1, 1,
                        0, 0, 0, 1;

  std::string ra_name = "Right Arm";
  std::string rl_name = "Right Leg";
  Arm right_arm(right_arm_torso_j0, ra_name);
  Arm::position curr_position = right_arm.FwdKinematics();
  Leg right_leg(right_leg_torso_j0, rl_name);
  
  return 0;
}

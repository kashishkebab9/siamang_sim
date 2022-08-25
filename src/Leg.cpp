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

}

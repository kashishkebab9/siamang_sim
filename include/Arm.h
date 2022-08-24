#pragma once

#include <iostream>
#include <cmath>
#include <math.h>
#include <Eigen/Dense>
#include <string>


class Arm {
  private:
    Eigen::Matrix4d offset_from_torso;
    std::string name;


  public:

    double l_1;
    double l_2;
    double l_3;

    double q0;
    double q1;
    double q2;

    struct position {
      double x;
      double y;
      double z;
    };

    struct joint_angles {
      double q0_;
      double q1_;
      double q2_;
    };
    
    Arm(Eigen::Matrix4d offset_from_torso_, std::string name_);

  
    //User should SetAngle before FwdKinematics
    void SetAngles(joint_angles config); 
    position FwdKinematics();
    
    

    





};

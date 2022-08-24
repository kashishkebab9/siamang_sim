# To compile the Siamath files:

g++ arm_kinematics.cpp -I $PATH_TO_EIGEN
# where $PATH_TO_EIGEN is the path to your local Eigen3 folder

# To compile main.cpp and arm.h:
g++ src/main.cpp src/Arm.cpp -Iinclude

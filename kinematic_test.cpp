#include "kinematic_model.h"
#include <iostream>
#include <array>
#include <eigen3/Eigen/Dense>


// Define main function for testing
int main() {
    // Define DH parameters for a 7-DoF robot (example values)
    std::array<double,8> alphaIn = {0., 0., 0., M_PI_4, 0., M_PI_2, M_PI / 3, M_PI / 5};
    std::array<double,8> aIn = {0.1, 0.1, 0.15, 0.16, 0.17, 0.18, 0.19, 0.};
    std::array<double,8> dIn = {0., 0., 0., 0., 0., 0., 0., 0.1};
    
    // Initialize the robot model
    RobotModel robot(aIn, dIn, alphaIn);

    // Define joint angles (example values)
    Eigen::Vector7d q;
    q << M_PI, M_PI_2, M_PI/3, M_PI_4, M_PI/5, M_PI/6, M_PI/7;

    // Prepare variables for output
    Eigen::Affine3d xOut;
    Eigen::Matrix67d JOut;

    // Compute forward kinematics and Jacobian
    robot.FwdKin(xOut, JOut, q);

    // Output results
    std::cout << "End-Effector Pose (xOut):\n" << xOut.matrix() << std::endl;
    std::cout << "\nJacobian Matrix (JOut):\n" << JOut << std::endl;

    return 0;
}

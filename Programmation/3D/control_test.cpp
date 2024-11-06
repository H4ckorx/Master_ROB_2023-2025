#include "control.h"
#include "trajectory_generation.h"
#include <iostream>

int main(){  
    // Define the DH parameters for the robot
    const std::array<double,8>  a     {      0,         0,         0,    0.0825,   -0.0825,         0,    0.0880,         0  };       
    const std::array<double,8>  d     { 0.3330,         0,    0.3160,         0,    0.3840,         0,         0,    0.1070  };         
    const std::array<double,8>  alpha {      0,   -1.5708,    1.5708,    1.5708,   -1.5708,    1.5708,    1.5708,         0  };           

    // Final time for the trajectory
    const double                tf    { 3.0                               };

    // Initial joint positions
    Eigen::Vector7d             q     {-0.479698, 0.0723513 ,-0.012116  ,-1.99778  , 1.32045  , 1.14296 ,-0.554182};

    // Jacobian matrix
    Eigen::Matrix<double,6,7>   J     { Eigen::Matrix<double,6,7>::Zero() };

    // Instantiate the robot model
    RobotModel                  rm    { a, d, alpha                       };    

    // Initial end-effector pose
    Eigen::Affine3d             xi    { Eigen::Affine3d::Identity()       };    
    rm.FwdKin(xi, J, q);  // Compute initial pose using forward kinematics

    // Define the desired final pose
    Eigen::Affine3d             xf    { xi                                };    
    xf.translation()(2)         += 0.14;  // Move target position upwards
    xf.linear().matrix()        = Eigen::AngleAxisd(-M_PI/4., Eigen::Vector3d::Unit(2)).matrix() * 
                                  Eigen::AngleAxisd(-M_PI/2., Eigen::Vector3d::Unit(0)).matrix() *
                                  xf.linear().matrix();

    // Create trajectory from initial pose to final pose
    Point2Point                 p2p   { xi, xf, tf                        };

    // Controller instantiation
    Controller controller(rm);

    // Set simulation parameters
    double timeStep = 0.02; // Time step for the simulation
    int numSteps = static_cast<int>(tf / timeStep);

    // Simulation loop
    for (int i = 0; i < numSteps; ++i) {
        double t = timeStep * i;

        // Desired position at time t
        Eigen::Affine3d Xd = p2p.X(t);

        // Desired feedforward velocity
        Eigen::Vector6d Dxd_ff = Eigen::Vector6d::Zero();

        // Calculate joint velocities to reach desired position
        Eigen::Vector7d Dqd = controller.Dqd(q, Xd, Dxd_ff);

        // Update joint angles based on joint velocities
        q = q + Dqd * timeStep;

        // Update forward kinematics for current joint angles
        rm.FwdKin(xi, J, q);

        // Output the current joint angles and end-effector position
        std::cout << "Time: " << t << " s\n";
        std::cout << "Joint angles (q1 to q7): ("
                  << q(0) << ", " << q(1) << ", " << q(2) << ", "
                  << q(3) << ", " << q(4) << ", " << q(5) << ", " << q(6) << ")\n";
        std::cout << "End-effector position (x, y, z): ("
                  << xi.translation()(0) << ", "
                  << xi.translation()(1) << ", "
                  << xi.translation()(2) << ")\n\n";
    }

    return 0;
}

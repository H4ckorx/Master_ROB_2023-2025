#include "kinematic_model.h"
#include <iostream>
#include <math.h>
int main(){
  // Compute the forward kinematics and jacobian matrix for 
  //      q =  M_PI/3.0,  M_PI_4
  // For a small variation of q, compute the variation on X and check dx = J . dq  
  RobotModel ROB(8,8);
  Eigen::Vector2d Q (M_PI/3,M_PI_4);
  Eigen::Vector2d xOut;
  Eigen::Matrix2d JOut;

  ROB.FwdKin(xOut,JOut,Q);
  std::cout << "For joint angles q1 = " << Q(0) * 180.0 / M_PI << " degrees, q2 = " << Q(1) * 180.0 / M_PI << " degrees:\n";
  std::cout << "End-effector position (x, y):\n" << xOut << std::endl;
  std::cout << "Jacobian matrix J:\n" << JOut << std::endl;

  Eigen::Vector2d dQ (0.01,0.01);

  Eigen::Vector2d dX = JOut * dQ;

  std::cout << "For a small variation dq = (0.05,0.05)\n";
  std::cout << "The predicted variation of position dX = Jout * dQ = \n" << dX << std::endl;

  Eigen::Vector2d New_Q = Q + dQ;
  Eigen::Vector2d xOut1;
  Eigen::Matrix2d JOut1;
  ROB.FwdKin(xOut1,JOut1,New_Q);
  Eigen::Vector2d dX1 = xOut1 - xOut;

  std::cout << "In reality the variation of position dX1 is \n" << dX1 << std::endl;

  return 0; 
}


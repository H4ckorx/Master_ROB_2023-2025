#include "control.h"
#include <iostream>
Controller::Controller(RobotModel &rmIn):
  model  (rmIn)
{
}

Eigen::Vector2d Controller::Dqd ( 
          const Eigen::Vector2d & q,
          const Eigen::Vector2d & xd,
          const Eigen::Vector2d & Dxd_ff
                      )
{  
  //TODO Compute joint velocities able to track the desired cartesian position

    model.FwdKin(X, J, q);  

    Eigen::Vector2d erreur = xd - X;

    dX_desired = kp * erreur + Dxd_ff;

    Eigen::Vector2d Dqd;

    Eigen::Matrix2d J_inv = J.completeOrthogonalDecomposition().pseudoInverse();
    Dqd = J_inv * dX_desired; 

    

  return Dqd;
}


#pragma once
#include "eigen3/Eigen/Dense"
#include "kinematic_model.h"

//! \brief Controller able to compute joint velocities for a given desired position
class Controller{
  public:
    //! \brief Controller of a 7-DoF robot described by a given model
    //! 
    //! \param model robot model
    Controller(RobotModel &model);

    //! \brief VJoint velocities for given joint position, desired cartesian position and feedforward term
    //!
    //! \return Dqd     joint velocities      
    //! \param  q       joint position
    //! \param  xd      desired cartesian position
    //! \param  Dxd_ff  joint velocity feedforward term
    Eigen::Vector7d   Dqd ( 
              const Eigen::Vector7d & q,
              const Eigen::Affine3d & xd,
              const Eigen::Vector6d & Dxd_ff
                          ); 
  private:
    RobotModel        model                                       ;     // Robot model
    const double      kp          { 6e+1                          };    // Feedback gain
    const double      kpR         { 6e+1                          };    // Feedback gain
    Eigen::Matrix67d  J           { Eigen::Matrix67d::Zero()      };    // Jacobian matrix
    Eigen::Vector6d   dX_desired  { Eigen::Vector6d ::Zero()      };    // Desired cartesian velocity
    Eigen::Affine3d   X           { Eigen::Affine3d ::Identity()  };    // Cartesian position
};

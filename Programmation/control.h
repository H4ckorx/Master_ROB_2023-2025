#pragma once
#include "eigen3/Eigen/Dense"
#include "kinematic_model.h"

//! \brief Controller able to compute joint velocities for a given desired position
class Controller{
  public:
    //! \brief Controller of a 2-DoF robot described by a given model
    //! 
    //! \param model robot model
    Controller(RobotModel &model);

    //! \brief VJoint velocities for given joint position, desired cartesian position and feedforward term
    //!
    //! \return Dqd     joint velocities      
    //! \param  q       joint position
    //! \param  xd      desired cartesian position
    //! \param  Dxd_ff  joint velocity feedforward term
    Eigen::Vector2d   Dqd ( 
              const Eigen::Vector2d & q,
              const Eigen::Vector2d & xd,
              const Eigen::Vector2d & Dxd_ff
                          ); 
  private:
    RobotModel        model                                  ;    // Robot model
    const double      kp          { 1e2                     };    // Feedback gain
    Eigen::Matrix2d   J           { Eigen::Matrix2d::Zero() };    // Jacobian matrix
    Eigen::Vector2d   dX_desired  { Eigen::Vector2d::Zero() };    // Desired cartesian velocity
    Eigen::Vector2d   X           { Eigen::Vector2d::Zero() };    // Cartesian position
};
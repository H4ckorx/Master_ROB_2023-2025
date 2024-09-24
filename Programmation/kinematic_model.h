#pragma once
#include "eigen3/Eigen/Dense"

//! \brief Kinematic model of a 2-DoF robot (Forward and differential)
class RobotModel{
  public:
    //! \brief Kinematic model with given link lengths 
    //!
    //! \param l1 length 1
    //! \param l2 length 2
    RobotModel(const double &l1In, const double &l2In);

    //! \brief Compute position and jacobian matrix for given joint position
    //!
    //! \param x Output cartesian position
    //! \param J Output jacobian matrix
    //! \param q joint position
    void FwdKin(Eigen::Vector2d &xOut, Eigen::Matrix2d &JOut, const Eigen::Vector2d & qIn);
  private:
    const double l1   ;   // length 1
    const double l2   ;   // length 2
};
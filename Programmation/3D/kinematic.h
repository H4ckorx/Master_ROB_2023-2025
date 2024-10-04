#pragma once
#include "eigen3/Eigen/Dense"
#include "trajectory_generation.h"

namespace Eigen{
  typedef Matrix<double,7,1> Vector7d;
  typedef Matrix<double,6,7> Matrix67d;
}

//! \brief Kinematic model of a 7-DoF robot (Forward and differential)
class RobotModel{
  public:
    //! \brief Kinematic model with given link lengths 
    //!
    //! \param a      DH parameters
    //! \param d      DH parameters
    //! \param alpha  DH parameters
    RobotModel(const std::array<double,8> &aIn, const std::array<double,8> &dIn, const std::array<double,8> &alphaIn);

    //! \brief Compute position and jacobian matrix for given joint position
    //!
    //! \param x Output cartesian position
    //! \param J Output jacobian matrix
    //! \param q joint position
    void FwdKin(Eigen::Affine3d &xOut, Eigen::Matrix67d &JOut, const Eigen::Vector7d & qIn);
  private:
    const std::array<double,8> &a     ;
    const std::array<double,8> &d     ;
    const std::array<double,8> &alpha ;
};

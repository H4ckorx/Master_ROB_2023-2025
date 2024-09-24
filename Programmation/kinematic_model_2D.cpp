#include "kinematic_model.h"

RobotModel::RobotModel(const double &l1In, const double &l2In):
  l1  (l1In),
  l2  (l2In)
{};

void RobotModel::FwdKin(Eigen::Vector2d &xOut, Eigen::Matrix2d &JOut, const Eigen::Vector2d & qIn){
  // TODO Implement the forward and differential kinematics
  double q1 = qIn(0);
  double q2 = qIn(1);

  xOut(0) = l1 * cos(q1) + l2 * cos(q1+q2);
  xOut(1) = l1 * sin(q1) + l2 * sin(q1+q2);

  JOut(0, 0) = -l1 * sin(q1) - l2 * sin(q1 + q2);  // ∂x/∂q1
  JOut(0, 1) = -l2 * sin(q1 + q2);                 // ∂x/∂q2
  JOut(1, 0) = l1 * cos(q1) + l2 * cos(q1 + q2);   // ∂y/∂q1
  JOut(1, 1) = l2 * cos(q1 + q2);                  // ∂y/∂q2
}

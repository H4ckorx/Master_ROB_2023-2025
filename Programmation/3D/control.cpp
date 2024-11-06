#include "control.h"
#include <iostream>

Controller::Controller(RobotModel &rmIn):
  model  (rmIn)
{
}

Eigen::Vector7d Controller::Dqd ( 
          const Eigen::Vector7d & q,
          const Eigen::Affine3d & xd,
          const Eigen::Vector6d & Dxd_ff
                      )
{  
    // 计算当前的笛卡尔位置和雅可比矩阵
    model.FwdKin(X, J, q);  

    // 计算位置误差
    Eigen::Vector3d position_error = xd.translation() - X.translation();

    // 计算旋转误差
    Eigen::Matrix3d R_desired = xd.rotation();
    Eigen::Matrix3d R_current = X.rotation();
    Eigen::Matrix3d R_error = R_desired * R_current.transpose();

    // 将旋转误差转换为角速度误差
    Eigen::AngleAxisd angle_axis_error(R_error);
    Eigen::Vector3d orientation_error = angle_axis_error.angle() * angle_axis_error.axis();

    // 合并位置误差和角度误差
    Eigen::Vector6d erreur;
    erreur.head<3>() = position_error;
    erreur.tail<3>() = orientation_error;

    // 计算所需的笛卡尔速度
    dX_desired = kp * erreur + Dxd_ff;

    // 使用雅可比矩阵的伪逆计算关节速度
    Eigen::Matrix<double, 7, 6> J_pseudo_inverse = J.completeOrthogonalDecomposition().pseudoInverse();
    Eigen::Vector7d Dqd = J_pseudo_inverse * dX_desired;

    return Dqd;
}

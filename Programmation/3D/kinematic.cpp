#include "kinematic_model.h"
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <cmath>

// RobotModel 构造函数
RobotModel::RobotModel(const std::array<double,8> &aIn, const std::array<double,8> &dIn, const std::array<double,8> &alphaIn):
  a {aIn},
  d {dIn},
  alpha {alphaIn}
{ 
};

// 计算正向运动学（FwdKin）和雅可比矩阵
void RobotModel::FwdKin(Eigen::Affine3d &xOut, Eigen::Matrix67d &JOut, const Eigen::Vector7d & qIn) {
    // 初始化末端执行器位姿为单位矩阵
    xOut = Eigen::Affine3d::Identity();

    // 存储每个关节的变换矩阵
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();  // 4x4 的齐次变换矩阵

    // 初始化雅可比矩阵
    JOut.setZero();  // 6x7 的雅可比矩阵

    // DH 参数以及关节角度遍历，假设 qIn 是各关节角度的向量
    for (int i = 0; i < 7; ++i) {
        double theta = qIn[i];  // 关节角度（输入）
        double ai = a[i];       // 连杆长度
        double di = d[i];       // 连杆偏移
        double alphai = alpha[i]; // 连杆扭转角
        
        // 计算每个关节的变换矩阵，根据 DH 参数
        Eigen::Matrix4d Ti;
        Ti << std::cos(theta), -std::sin(theta), 0, ai-1,
              std::sin(theta) * std::cos(alphai), std::cos(theta) * std::cos(alphai), -std::sin(alphai), -std::sin(alphai) * di,
              std::sin(theta) * std::sin(alphai), std::cos(theta) * std::sin(alphai), std::cos(alphai), std::cos(alphai) * di,
              0, 0, 0, 1;


        // 更新总的变换矩阵
        T = T * Ti;

        // 提取出末端执行器的旋转和平移部分
        Eigen::Matrix3d R = T.block<3,3>(0, 0);  // 旋转部分
        Eigen::Vector3d p = T.block<3,1>(0, 3);  // 平移部分

        // 计算雅可比矩阵的列
        Eigen::Vector3d z_i = T.block<3,1>(0, 2);  // 当前关节的 z 轴方向
        Eigen::Vector3d p_i = p;  // 当前关节的位置
        Eigen::Vector3d p_e = T.block<3,1>(0, 3);  // 末端执行器的位置

        // 雅可比矩阵线速度部分
        JOut.block<3,1>(0,i) = z_i.cross(p_e - p_i);  // 线速度部分

        // 雅可比矩阵角速度部分
        JOut.block<3,1>(3,i) = z_i;  // 角速度部分
    }

    // 最后，设置末端执行器位姿输出
    xOut.matrix() = T;  // 赋值 4x4 变换矩阵
}

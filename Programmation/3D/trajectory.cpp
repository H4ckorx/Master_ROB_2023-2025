#include "trajectory_generation.h"

Polynomial::Polynomial(){};

Polynomial::Polynomial(const double &piIn, const double &pfIn, const double & DtIn){
};

void Polynomial::update(const double &piIn, const double &pfIn, const double & DtIn){
    // 更新多项式的系数
    // Update polynomial coefficients
    pi = piIn;
    pf = pfIn;
    Dt = DtIn;
    
    // 计算五阶多项式的系数
    // Calculate the coefficients of the 5th-degree polynomial
    a[0] = pi;
    a[1] = 0;
    a[2] = 0;
    a[3] = 10 * (pf - pi) / (Dt * Dt * Dt);
    a[4] = -15 * (pf - pi) / (Dt * Dt * Dt * Dt);
    a[5] = 6 * (pf - pi) / (Dt * Dt * Dt * Dt * Dt);
};

const double Polynomial::p(const double &t) {
    return a[0] + a[1] * t + a[2] * t * t + a[3] * t * t * t + a[4] * t * t * t * t + a[5] * t * t * t * t * t;
}

// 计算指定时间 t 下的速度
// Compute the velocity at the given time t
const double Polynomial::dp(const double &t) {
    return a[1] + 2 * a[2] * t + 3 * a[3] * t * t + 4 * a[4] * t * t * t + 5 * a[5] * t * t * t * t;
}

#include "trajectory_generation.h"
///1. X_i.rotation() 和 X_f.rotation()   X_i 和 X_f 是 Eigen::Affine3d 类型的变量，表示机器人的初始位姿和最终位姿。
///Affine3d 是一个 4x4 的齐次变换矩阵，包含了平移和旋转信息：T=[ [R 0] [t 1]]  其中：R 是 3x3 的旋转矩阵，表示方向（旋转部分）。t 是 3x1 的平移向量。
///X_i.rotation() 和 X_f.rotation() 会提取 X_i 和 X_f 中的 旋转部分 Ri 和 Rf ，它们是 Eigen::Matrix3d 类型（即 3x3 矩阵）。这一步的作用是从初始和最终位姿中提取出旋转矩阵。
///Eigen::AngleAxisd 将旋转矩阵转换为轴角表示，并从中提取旋转轴和旋转角度。



// Point2Point 类构造函数
// Point2Point class constructor
Point2Point::Point2Point(const Eigen::Affine3d & X_i, const Eigen::Affine3d & X_f, const double & DtIn) {
    Dt = DtIn;
    
    // 提取初始和最终位置（平移）
    // Extract initial and final positions (translation)
    Eigen::Vector3d position_i = X_i.translation();
    Eigen::Vector3d position_f = X_f.translation();
    
    // 初始化 x, y, z 方向的多项式轨迹生成
    // Initialize polynomial trajectory generation in x, y, z directions
    polx.update(position_i.x(), position_f.x(), Dt);
    poly.update(position_i.y(), position_f.y(), Dt);
    polz.update(position_i.z(), position_f.z(), Dt);

    // 初始化旋转部分（使用 AngleAxis 表示旋转）
    // Initialize the rotation section (using AngleAxis for rotation)
    Eigen::AngleAxisd rotation_i(X_i.rotation());
    Eigen::AngleAxisd rotation_f(X_f.rotation());
    
    R0 = rotation_i.toRotationMatrix();
    Eigen::AngleAxisd delta_rot(R0.transpose() * rotation_f); // 相对旋转 // Relative rotation
    rot_aa = delta_rot; // 存储相对旋转 // Store relative rotation
    axis = delta_rot.axis(); // 旋转轴 // Rotation axis
    pol_angle.update(0.0, delta_rot.angle(), Dt); // 设置旋转角度的轨迹 // Set the trajectory of the rotation angle
}

// 计算指定时间 t 下的位姿
// Compute the pose at the given time t
Eigen::Affine3d Point2Point::X(const double & time) {
    Eigen::Affine3d result = Eigen::Affine3d::Identity();

    // 计算平移部分 (x, y, z)
    // Compute the translation part (x, y, z)
    result.translation().x() = polx.p(time);
    result.translation().y() = poly.p(time);
    result.translation().z() = polz.p(time);

    // 计算旋转部分
    // Compute the rotation part
    double angle = pol_angle.p(time); // 获取在当前时间的旋转角度 // Get the rotation angle at the current time
    Eigen::AngleAxisd rotation(angle, axis); // 根据旋转轴和角度生成旋转 // Generate rotation based on the axis and angle
    result.linear() = R0 * rotation.toRotationMatrix() ; // 应用旋转到初始方向 // Apply rotation to the initial direction

    return result;
}

// 计算指定时间 t 下的速度
// Compute the velocity at the given time t
Eigen::Vector6d Point2Point::dX(const double & time) {
    Eigen::Vector6d velocity;
    
    // 计算线速度 (x, y, z)
    // Compute the linear velocity (x, y, z)
    velocity(0) = polx.dp(time);
    velocity(1) = poly.dp(time);
    velocity(2) = polz.dp(time);
    
    // 计算角速度
    // Compute the angular velocity
    double angular_speed = pol_angle.dp(time); // 旋转角速度 // Rotational angular velocity
    Eigen::Vector3d angular_velocity = angular_speed * axis; // 使用旋转轴和角速度计算 // Compute using the rotation axis and angular velocity
    
    velocity.tail<3>() = angular_velocity; // 将角速度加入到速度向量的最后3个元素 // Add angular velocity to the last 3 elements of the velocity vector
    
    return velocity;
}

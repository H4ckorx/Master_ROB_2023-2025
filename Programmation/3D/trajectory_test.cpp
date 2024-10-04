#include "trajectory_generation.h"
#include <iostream>
#include <eigen3/Eigen/Dense>

int main() {
    // 设置初始位姿和最终位姿（Affine3d 包含旋转和平移）
    // Set initial and final poses (Affine3d contains both rotation and translation)
    Eigen::Affine3d X_i = Eigen::Affine3d::Identity();
    Eigen::Affine3d X_f = Eigen::Affine3d::Identity();

    // 设置初始位置和平移 (设置平移部分)
    // Set the initial position and translation (translation part)
    X_i.translation() << 0.0, 0.0, 0.0;  // 初始位置 (0, 0, 0) // Initial position (0, 0, 0)
    X_f.translation() << 10.0, 10.0, 10.0; // 最终位置 (10, 10, 10) // Final position (10, 10, 10)

    // 设置初始旋转和最终旋转 (沿 z 轴旋转 90 度)
    // Set the initial and final rotations (rotate 90 degrees along the z-axis)
    X_i.rotate(Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ())); // 初始没有旋转 // No initial rotation
    X_f.rotate(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ())); // 旋转 90 度 // Rotate 90 degrees

    // 设定运动时间 Dt
    // Set the movement time Dt
    double Dt = 10.0;

    // 初始化轨迹生成
    // Initialize trajectory generation
    Point2Point trajectory(X_i, X_f, Dt);

    // 定义时间步长
    // Define the time step
    double timestep = 0.5;
    for (double t = 0.0; t <= Dt; t += timestep) {
        // 计算当前时间的位姿 (Affine3d 形式，包含平移和旋转)
        // Calculate the pose at the current time (Affine3d format, including translation and rotation)
        Eigen::Affine3d position = trajectory.X(t);

        // 计算当前时间的速度 (包括线速度和角速度)
        // Calculate the velocity at the current time (including linear and angular velocities)
        Eigen::Vector6d velocity = trajectory.dX(t);

        // 输出当前时间的位姿和速度
        // Output the pose and velocity at the current time
        std::cout << "Time: " << t << "s\n";
        std::cout << "Position: (" 
                  << position.translation().x() << ", "
                  << position.translation().y() << ", "
                  << position.translation().z() << ")\n";

        std::cout << "Rotation Matrix:\n" << position.rotation() << "\n";

        std::cout << "Linear Velocity: (" 
                  << velocity(0) << ", " << velocity(1) << ", " << velocity(2) << ")\n";
        std::cout << "Angular Velocity: (" 
                  << velocity(3) << ", " << velocity(4) << ", " << velocity(5) << ")\n";
        std::cout << "-------------------------------------------\n";
    }

    // 检查初始和最终时间点的速度
    // Check the velocities at the initial and final time points
    Eigen::Vector6d v0 = trajectory.dX(0.0); // 初始速度 // Initial velocity
    Eigen::Vector6d vFinal = trajectory.dX(Dt); // 最终速度 // Final velocity

    std::cout << "\nInitial velocity: (" 
              << v0(0) << ", " << v0(1) << ", " << v0(2) << ", "
              << v0(3) << ", " << v0(4) << ", " << v0(5) << ")\n";

    std::cout << "Final velocity: (" 
              << vFinal(0) << ", " << vFinal(1) << ", " << vFinal(2) << ", "
              << vFinal(3) << ", " << vFinal(4) << ", " << vFinal(5) << ")\n";

    return 0;
}



// **************** 速度残差与雅可比 ****************/
const Eigen::Vector3d v_wheel_global = R_bw * v_wheel;  // 轮速转全局坐标系
const Eigen::Vector3d residual_v = v_wheel_global - bodystate_cur_.vel;  // 速度残差

Eigen::Matrix<double, 3, 15> H_wheel_vel = Eigen::Matrix<double, 3, 15>::Zero();
H_wheel_vel.block<3, 3>(0, 0) = skew_symmetric(v_wheel_global);  // 对姿态的导数
H_wheel_vel.block<3, 3>(0, 6) = -Eigen::Matrix3d::Identity();     // 对速度的导数

// **************** 位置残差与雅可比 ****************/
const Eigen::Vector3d wheel_obs = trans_delta + bodystate_pre_lidar_update_.pose.translation();
const Eigen::Vector3d residual_p = wheel_obs - bodystate_cur_.pose.translation();  // 位置残差

Eigen::Matrix<double, 3, 15> H_wheel_pos = Eigen::Matrix<double, 3, 15>::Zero();
H_wheel_pos.block<3, 3>(0, 3) = -Eigen::Matrix3d::Identity();  // 对平移的导数

// **************** 总残差与总雅可比 ****************/
Eigen::Vector6d residual_wheel;
residual_wheel << residual_v, residual_p;  // 拼接为6维残差（先速度后位置）

Eigen::Matrix<double, 6, 15> H_wheel_total;
H_wheel_total << H_wheel_vel, H_wheel_pos;  // 拼接总雅可比

// **************** 总观测噪声 ****************/
Eigen::Matrix3d R_wheel_vel = Eigen::Vector3d(0.05, 0.5, 50).cwiseAbs2().asDiagonal();
Eigen::Matrix3d R_wheel_pos = Eigen::Vector3d(0.1, 0.1, 0.1).cwiseAbs2().asDiagonal();  // 需根据轮速计精度调整
Eigen::Matrix<double, 6, 6> R_wheel_total;
R_wheel_total.setZero();
R_wheel_total.block<3, 3>(0, 0) = R_wheel_vel;
R_wheel_total.block<3, 3>(3, 3) = R_wheel_pos;

// **************** 卡尔曼增益 ****************/
Eigen::Matrix<double, 15, 15> I = Eigen::Matrix<double, 15, 15>::Identity();
Eigen::Matrix<double, 6, 15> K_wheel_total = Cov_ * H_wheel_total.transpose() *
    (H_wheel_total * Cov_ * H_wheel_total.transpose() + R_wheel_total).inverse();


// **************** 更新状态误差 δx_ ****************/
delta_x_ += K_wheel_total * residual_wheel;  // K是6×15，残差是6×1，结果15×1

// **************** 反馈更新状态（复用现有函数） ****************/
stateFeedback();  // 需确保stateFeedback能处理15维δx_

// **************** 更新协方差矩阵 ****************/
Cov_ = (I - K_wheel_total * H_wheel_total) * Cov_ *
    (I - K_wheel_total * H_wheel_total).transpose() +
    K_wheel_total * R_wheel_total * K_wheel_total.transpose();

// **************** 重置轮速积分状态（可选） ****************/
wheel_integration_state_ = BodyState();
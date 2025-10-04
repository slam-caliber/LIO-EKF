// MIT License
//
// Copyright (c) 2024 Yibin Wu, Tiziano Guadagnino
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
#include "lio_ekf.hpp"
#include "kiss_icp/core/Deskew.hpp"
#include "kiss_icp/core/Preprocessing.hpp"
#include "kiss_icp/pipeline/KissICP.hpp"
#include "lio_types.hpp"
#include "rotation.hpp"
#include "unscendent_transform.hpp"
#include <Eigen/Sparse>
#include <algorithm>
#include <execution>
#include <fstream>
#include <ros/ros.h>
#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>
#include <tbb/parallel_reduce.h>

namespace {
Eigen::Vector3d SIGMA_POINT =
    Eigen::Vector3d(0.01, 0.05 * lio_ekf::D2R, 0.05 * lio_ekf::D2R);
Eigen::Matrix3d POINT_NOISE_COV = SIGMA_POINT.cwiseAbs2().asDiagonal();
} // namespace

inline double square(double x) { return x * x; }

Eigen::Matrix3d skew_symmetric(const Eigen::Vector3d& v) {
    Eigen::Matrix3d mat;
    mat << 0, -v[2], v[1],
           v[2], 0, -v[0],
          -v[1], v[0], 0;
    return mat;
}

namespace lio_ekf {

auto create_covariance_block = [](const auto &v1) {
  return v1.cwiseAbs2().asDiagonal();
};

void LIOEKF::init() {

  imu_t_ = 0.0;
  lidar_t_ = 0.0;

  // resize covariance matrix, system noise matrix, and system error state
  // matrix

  Cov_.setZero();
  Imu_Prediction_Covariance_.setZero();
  State_Noise_Cov_.setZero();
  delta_x_.setZero();

  // initialize noise matrix
  auto imunoise = liopara_.imunoise;

  State_Noise_Cov_.block(ANGEL_RANDOMWALK_ID, ANGEL_RANDOMWALK_ID, 3, 3) =
      create_covariance_block(imunoise.angle_randomwalk);
  State_Noise_Cov_.block(VEL_RANDOMWALK_ID, VEL_RANDOMWALK_ID, 3, 3) =
      create_covariance_block(imunoise.velocity_randomwalk);
  State_Noise_Cov_.block(GYRO_BIAS_STD_ID, GYRO_BIAS_STD_ID, 3, 3) =
      2 / imunoise.correlation_time *
      create_covariance_block(imunoise.gyrbias_std);
  State_Noise_Cov_.block(ACC_BIAS_STD_ID, ACC_BIAS_STD_ID, 3, 3) =
      2 / imunoise.correlation_time *
      create_covariance_block(imunoise.accbias_std);

  // set initial state (position, velocity, attitude and IMU error) and
  // covariance
  navStateInitialization(liopara_.initstate, liopara_.initstate_std);

  is_first_imu_ = true;
  is_first_lidar_ = true;
  is_first_wheel_ = true;
 }

void LIOEKF::navStateInitialization(const NavState &initstate,
                                    const NavState &initstate_std) {

  // initialize imu error
  imuerror_ = initstate.imuerror;

  // initialize covariance
  ImuError imuerror_std = initstate_std.imuerror;

  Cov_.block(POS_ID, POS_ID, 3, 3) = create_covariance_block(initstate_std.pos);
  Cov_.block(VEL_ID, VEL_ID, 3, 3) = create_covariance_block(initstate_std.vel);
  Cov_.block(ATT_ID, ATT_ID, 3, 3) =
      create_covariance_block(initstate_std.euler);
  Cov_.block(GYRO_BIAS_ID, GYRO_BIAS_ID, 3, 3) =
      create_covariance_block(imuerror_std.gyrbias);
  Cov_.block(ACC_BIAS_ID, ACC_BIAS_ID, 3, 3) =
      create_covariance_block(imuerror_std.accbias);
}

void LIOEKF::initFirstLiDAR(const int lidarUpdateFlag) {
  Sophus::SE3d initLidarpose_w = Sophus::SE3d();
  const auto &lidar_to_imu_extrinsic = Sophus::SE3d(liopara_.Trans_lidar_imu);
  switch (lidarUpdateFlag) {
  case 1: {
    initLidarpose_w = bodystate_pre_.pose * lidar_to_imu_extrinsic;
    break;
  }
  case 2: {
    initLidarpose_w = bodystate_cur_.pose * lidar_to_imu_extrinsic;
    break;
  }
  case 3: {
    const auto delta_pose = bodystate_pre_.pose.inverse() * bodystate_cur_.pose;
    double relative_lidar_timestamp = (lidar_t_ - imupre_.timestamp) /
                                      (imucur_.timestamp - imupre_.timestamp);
    const auto interpolated_vector_pose =
        relative_lidar_timestamp * delta_pose.log();
    const Sophus::SE3d interpolated_pose =
        bodystate_pre_.pose * Sophus::SE3d::exp(interpolated_vector_pose);
    initLidarpose_w = interpolated_pose * lidar_to_imu_extrinsic;
    break;
  }
  }

  lio_map_.Update(curpoints_, initLidarpose_w);
  is_first_lidar_ = false;
  last_update_t_ = lidar_t_;
  first_lidar_t = lidar_t_;
}

void LIOEKF::newWheelProcess(){

  if (is_first_wheel_) {
    bodystate_pre_ = bodystate_cur_;
    wheelpre_ = wheelcur_;
    wheel_t_ = wheelcur_.timestamp;
    is_first_wheel_ = false;
    return;
  }
  //std::cout<<"newWheelProcess"<<std::endl;
  // 轮速运动模型积分
  double dt = wheelcur_.dt;
  //if (dt <= 0) continue;
  
  // 获取当前姿态
  Eigen::Matrix3d R = bodystate_cur_.pose.rotationMatrix();
  
  // 轮速测量值（车身坐标系）
  Eigen::Vector3d v_body(wheelcur_.linear_velocity, 0.0, 0.0);
  double omega_z = wheelcur_.angular_velocity;
  
  // 转换到全局坐标系
  Eigen::Vector3d v_global = R * v_body;
  
  // 更新位置
  wheel_integration_state_.pose.translation() += v_global * dt;

}

void LIOEKF::newImuProcess() {

  if (is_first_imu_) {
    bodystate_pre_ = bodystate_cur_;
    imupre_ = imucur_;
    imu_t_ = imucur_.timestamp;
    is_first_imu_ = false;
    return;
  }

  // set update time as the lidar time stamp
  double updatetime = lidar_t_;

  int lidarUpdateFlag = 0;

  if (lidar_t_ > last_update_t_ + 0.001 || is_first_lidar_)
    lidarUpdateFlag =
        isToUpdate(imupre_.timestamp, imucur_.timestamp, updatetime);
  // determine if we should do  update

  switch (lidarUpdateFlag) {
  case 0: {
    // only propagate navigation state
    //imu 200hz, lidar 10hz; imu做20次积分，case改变一次，进行一次下面的lidar更新
    //std::cout<<"case 0:only propagate navigation state"<<std::endl;
    statePropagation(imupre_, imucur_);
    break;
  }
  case 1: {
    // lidardata is near to the previous imudata, we should firstly do lidar
    // update
    // std::cout<<"case 1:lidardata is near to the previous imudata, we should firstly do lidar"<<std::endl;
    if (is_first_lidar_) {
      initFirstLiDAR(lidarUpdateFlag);
    } else {
      wheelUpdate();
      lidarUpdate();
      
    }

    lidar_updated_ = true;

    bodystate_pre_ = bodystate_cur_;
    statePropagation(imupre_, imucur_);
    break;
  }
  case 2: {
    // lidardata is near current imudata, we should firstly propagate navigation
    // state
    //std::cout<<"case 2: lidardata is near to the previous imudata, we should firstly do lidar"<<std::endl;
    statePropagation(imupre_, imucur_);
    if (is_first_lidar_) {
      initFirstLiDAR(lidarUpdateFlag);
    } else {
      wheelUpdate();
      lidarUpdate();
      
    }

    lidar_updated_ = true;

    break;
  }
  case 3: {
    // lidardata is between the two imudata, we interpolate current imudata to
    // lidar time
    //std::cout<<"case 3: lidardata is between the two imudata, we interpolate current imudata to"<<std::endl;
    IMU midimu;
    imuInterpolate(imupre_, imucur_, updatetime, midimu);

    // propagate navigation state for the first half imudata
    statePropagation(imupre_, midimu);

    // do lidar position update at the whole second and feedback system states
    if (is_first_lidar_) {
      initFirstLiDAR(lidarUpdateFlag);
    } else {
      wheelUpdate();
      lidarUpdate();
      
    }

    lidar_updated_ = true;
    // propagate navigation state for the second half imudata
    bodystate_pre_ = bodystate_cur_;
    statePropagation(midimu, imucur_);
    break;
  }
  }

  // check diagonal elements of current covariance matrix
  checkStateCov();

  // update system state and imudata at the previous epoc
  bodystate_pre_ = bodystate_cur_;
  


  imupre_ = imucur_;
}

void LIOEKF::statePropagation(IMU &imupre, IMU &imucur) {

  // debug_<<"statePropagation start"<<std::endl;
  // compensate imu error to 'imucur', 'imupre' has been compensated
  imuCompensate(imucur, imuerror_);

  // update imustate(mechanization)
  insMechanization(bodystate_pre_, bodystate_cur_, imupre, imucur);

  // system noise propagate, phi-angle error model for attitude error

  Eigen::Matrix15d State_Transition_Mat, Jacobian, Process_Noise_Cov;
  Eigen::Matrix15_12d Noise_Driven_Mat;

  State_Transition_Mat.setIdentity();
  Jacobian.setZero();
  Process_Noise_Cov.setZero();
  Noise_Driven_Mat.setZero();

  // position error
  Jacobian.block(POS_ID, VEL_ID, 3, 3) = Eigen::Matrix3d::Identity();

  // velocity error
  Jacobian.block(VEL_ID, ATT_ID, 3, 3) = Rotation::skewSymmetric(
      bodystate_pre_.pose.rotationMatrix() * imucur.linear_acceleration);
  Jacobian.block(VEL_ID, ACC_BIAS_ID, 3, 3) =
      bodystate_pre_.pose.rotationMatrix();

  // attitude error
  Jacobian.block(ATT_ID, GYRO_BIAS_ID, 3, 3) =
      -1.0 * bodystate_pre_.pose.rotationMatrix();

  // system noise driven matrix
  Noise_Driven_Mat.block(VEL_ID, VEL_RANDOMWALK_ID, 3, 3) =
      bodystate_pre_.pose.rotationMatrix();
  Noise_Driven_Mat.block(ATT_ID, ANGEL_RANDOMWALK_ID, 3, 3) =
      bodystate_pre_.pose.rotationMatrix();
  Noise_Driven_Mat.block(GYRO_BIAS_ID, GYRO_BIAS_STD_ID, 3, 3) =
      Eigen::Matrix3d::Identity();
  Noise_Driven_Mat.block(ACC_BIAS_ID, ACC_BIAS_STD_ID, 3, 3) =
      Eigen::Matrix3d::Identity();

  // compute the state transition matrix
  State_Transition_Mat.setIdentity();
  State_Transition_Mat = State_Transition_Mat + Jacobian * imucur.dt;

  // compute system propagation noise
  Process_Noise_Cov = Noise_Driven_Mat * State_Noise_Cov_ *
                      Noise_Driven_Mat.transpose() * imucur.dt;
  Process_Noise_Cov = (State_Transition_Mat * Process_Noise_Cov *
                           State_Transition_Mat.transpose() +
                       Process_Noise_Cov) /
                      2;
  Process_Noise_Cov += 1e-8 * Eigen::Matrix15d::Identity();

  ekfPredict(State_Transition_Mat, Process_Noise_Cov);
}

auto LIOEKF::processScan() {
  Sophus::SE3d lidar_to_imu = Sophus::SE3d(liopara_.Trans_lidar_imu);
  Sophus::SE3d previous_pose_scan = bodystate_pre_.pose * lidar_to_imu;
  Sophus::SE3d current_pose_scan = bodystate_cur_.pose * lidar_to_imu;
  curpoints_w_ = kiss_icp::DeSkewScan(curpoints_, timestamps_per_points_,
                                      previous_pose_scan, current_pose_scan);
  const auto cropped_frame = kiss_icp::Preprocess(
      curpoints_w_, liopara_.max_range, liopara_.min_range);
  auto [source, frame_downsample] = Voxelize(cropped_frame);
  auto source_in_imu_frame = source;
  keypoints_w_ = source;
  TransformPoints(lidar_to_imu.matrix(), source_in_imu_frame);

  const Eigen::Matrix4d &lidar_to_imu_origin = liopara_.Trans_lidar_imu_origin;
  TransformPoints(lidar_to_imu_origin, keypoints_w_);
  TransformPoints(lidar_to_imu_origin, curpoints_w_);

  return std::make_tuple(source_in_imu_frame, frame_downsample);
}


void LIOEKF::wheelUpdate() {
  

  Eigen::Vector3d trans_delta = wheel_integration_state_.pose.translation();
  Eigen::Vector3d rot_delta = wheel_integration_state_.pose.so3().log();
  Eigen::Vector3d v_wheel(wheelcur_.linear_velocity, 0.0, 0.0);

  const Eigen::Vector3d& v_global = bodystate_cur_.vel;  // 全局速度（O_w下）
  const Eigen::Matrix3d R_bw = bodystate_cur_.pose.rotationMatrix();  // O_b→O_w的旋转矩阵
  const Eigen::Quaterniond q_bw = Eigen::Quaterniond(R_bw);  // O_b→O_w的四元数

  Eigen::Vector3d v_whell_in_global = R_bw * v_wheel;  // 公式：v_w^body = R_{b→w}^T * v_w
  const Eigen::Vector3d residual_v = (v_whell_in_global - v_global);

  // -------------------------- 步骤3：构建观测雅可比 --------------------------
  Eigen::Matrix3d v_whell_in_global_skew = skew_symmetric(v_whell_in_global);
  Eigen::Matrix3_15d H_v = Eigen::Matrix3_15d::Zero();
  H_v.block<3, 3>(0, 3) = -Eigen::Matrix3d::Identity();
  H_v.block<3, 3>(0, 6) = v_whell_in_global_skew;

  const Eigen::Vector3d translation_global = bodystate_cur_.pose.translation();

  Eigen::Vector3d wheel_obs = trans_delta + bodystate_pre_lidar_update_.pose.translation();

  const Eigen::Vector3d residual_p= ( wheel_obs - translation_global);
  Eigen::Matrix3_15d H_p = Eigen::Matrix3_15d::Zero();
  H_p.block<3, 3>(0, 0) = -Eigen::Matrix3d::Identity();
  H_p.block<3, 3>(0, 6) = v_whell_in_global_skew;



  // **************** 总残差与总雅可比 ****************/
  Eigen::Vector6d residual_wheel;
  residual_wheel << residual_p, residual_v;  // 拼接为6维残差（先速度后位置）

  Eigen::Matrix<double, 6, 15> H_wheel_total;
  H_wheel_total << H_p, H_v;  // 拼接总雅可比
  std::cout<<"residual_wheel"<<residual_wheel<<std::endl;

    // **************** 总观测噪声 ****************/
  Eigen::Matrix3d R_wheel_pos = Eigen::Vector3d(0.01, 0.01, 200).cwiseAbs2().asDiagonal();  // 需根据轮速计精度调整
  Eigen::Matrix3d R_wheel_vel = Eigen::Vector3d(0.05, 0.5, 50).cwiseAbs2().asDiagonal();
  
  Eigen::Matrix<double, 6, 6> R_wheel_total;
  R_wheel_total.setZero();
  R_wheel_total.block<3, 3>(0, 0) = R_wheel_pos;
  R_wheel_total.block<3, 3>(3, 3) = R_wheel_vel;

  Eigen::Matrix<double, 15, 15> I = Eigen::Matrix<double, 15, 15>::Identity();
  Eigen::Matrix<double, 15, 6> K_wheel_total = Cov_ * H_wheel_total.transpose() *
    (H_wheel_total * Cov_ * H_wheel_total.transpose() + R_wheel_total).inverse();

  // **************** 更新状态误差 δx_ ****************/
    delta_x_ += K_wheel_total * residual_wheel;  // K是6×15，残差是6×1，结果15×1

  // // std::cout<<"K_wheel_total:"<<K_wheel_total<<std::endl;

  // // // **************** 反馈更新状态（复用现有函数） ****************/
    stateFeedback();  // 需确保stateFeedback能处理15维δx_

  // // **************** 更新协方差矩阵 ****************/
  Cov_ = (I - K_wheel_total * H_wheel_total) * Cov_ *
      (I - K_wheel_total * H_wheel_total).transpose() +
      K_wheel_total * R_wheel_total * K_wheel_total.transpose();

  // -------------------------- 步骤4：构建卡尔曼增益 --------------------------
  //Eigen::Matrix<double, 15, 3> K_wheel = Cov_ * H.transpose() * (H * Cov_ * H.transpose() + R_wheel).inverse();

  // 8. 更新状态误差 delta_x_
 // delta_x_ += K_wheel * residual;


  // 9. 反馈更新状态（位置、速度、姿态、IMU零偏）
  //stateFeedback(); // 复用现有的误差反馈函数

  //const Eigen::Vector3d& v_global_wheel_update = bodystate_cur_.vel;

  // 10. 更新协方差矩阵
  //Eigen::Matrix<double, 15, 15> I = Eigen::Matrix<double, 15, 15>::Identity();
 // Cov_ = (I - K_wheel * H) * Cov_ * (I - K_wheel * H).transpose() + K_wheel * R_wheel * K_wheel.transpose();


  wheel_integration_state_ = BodyState();
  //std::cout<<"重置wheel_integration_state_"<<std::endl;
}

void LIOEKF::checkLidarDegeneration(const Eigen::Matrix15d& HTRH)
{
       // 提取位置部分的信息矩阵 (3x3)
  Eigen::Matrix3d position_info = HTRH.block<3, 3>(0, 0);
  
  // 特征值分解
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigensolver(position_info);
  Eigen::Vector3d eigenvalues = eigensolver.eigenvalues();
  Eigen::Matrix3d eigenvectors = eigensolver.eigenvectors();
  
  // 计算条件数（最大特征值/最小特征值）
  double cond_number = eigenvalues.maxCoeff() / eigenvalues.minCoeff();


  std::cout << "Relative eigenvalues (to max): [";
  for (int i = 0; i < 3; ++i) {
    std::cout << eigenvalues[i];
    if (i < 5) std::cout << ", ";
  }
  std::cout << "]" << std::endl;
  
  // 检测退化条件
  bool is_degenerate = false;
  if (cond_number > 1000) { // 条件数阈值
    is_degenerate = true;
    std::cout << "WARNING: Degenerate case detected! Condition number: " 
              << cond_number << std::endl;
  }
  
  // 检查特征值分布
  double eigenvalue_sum = eigenvalues.sum();
  for (int i = 0; i < 3; ++i) {
    double ratio = eigenvalues(i) / eigenvalue_sum;
    if (ratio < 0.01) { // 某个方向信息量很小
      std::cout << "Weak direction: " << eigenvectors.col(i).transpose() 
                << " with ratio: " << ratio << std::endl;
    }
  }

  
}

void LIOEKF::setupCorridorSpecificWeights(Eigen::Matrix3d& R_inv, 
                                     const Eigen::Vector3d& direction, 
                                     double noise_scale)
{
  double lateral_weight_boost = 5.0;
  double vertical_weight_boost = 5.0;
// 找出与前进方向最对齐的轴
    Eigen::Vector3d abs_direction = direction.cwiseAbs();
    int max_axis = 0;
    abs_direction.maxCoeff(&max_axis);
    
    // 确定另外两个轴
    int lateral_axis = (max_axis + 1) % 3;  // 侧向轴
    int vertical_axis = (max_axis + 2) % 3; // 垂直轴
    
    // 调整权重：
    // 1. 降低前进方向权重（增大噪声）
    R_inv(max_axis, max_axis) /= noise_scale;
    
    // 2. 增加侧向权重（降低噪声）
    R_inv(lateral_axis, lateral_axis) *= lateral_weight_boost;
    
    // 3. 增加垂直权重（降低噪声）
    R_inv(vertical_axis, vertical_axis) *= vertical_weight_boost;
    
    std::cout << "Corridor weight adjustment:" << std::endl;
    std::cout << "Forward axis " << max_axis << ": weight reduced by " << noise_scale << std::endl;
    std::cout << "Lateral axis " << lateral_axis << ": weight increased by " << lateral_weight_boost << std::endl;
    std::cout << "Vertical axis " << vertical_axis << ": weight increased by " << vertical_weight_boost << std::endl;
}

void LIOEKF::lidarUpdate() {

  auto [source, frame_downsample] = processScan();
  Eigen::Matrix6d imu_pose_covariance = Eigen::Matrix6d::Identity();
  imu_pose_covariance.block<3, 3>(0, 0) =
      Imu_Prediction_Covariance_.block<3, 3>(0, 0);
  imu_pose_covariance.block<3, 3>(3, 3) =
      Imu_Prediction_Covariance_.block<3, 3>(6, 6);
  imu_pose_covariance.block<3, 3>(0, 3) =
      Imu_Prediction_Covariance_.block<3, 3>(0, 6);
  imu_pose_covariance.block<3, 3>(3, 0) =
      Imu_Prediction_Covariance_.block<3, 3>(6, 0);
  const auto relative_pose =
      bodystate_pre_.pose.inverse() * bodystate_cur_.pose;


  const auto initial_guess = bodystate_cur_.pose;
  auto square = [](const double &x) { return x * x; };
  double uncertanty_motion =
      propagateUscendent(relative_pose, imu_pose_covariance);
  double map_uncertanty =
      square(liopara_.voxel_size / std::sqrt(liopara_.max_points_per_voxel));
  double range_uncertanty = square(0.05);
  // double max_correspondence_distance = 1.0;
  double max_correspondence_distance =
      6 * std::sqrt(uncertanty_motion + map_uncertanty + range_uncertanty);

  Eigen::Vector3d forward_direction_body = Eigen::Vector3d::UnitX(); // 假设x是前进方向
  
  // 将前进方向转换到全局坐标系
  Eigen::Vector3d forward_direction_global = bodystate_cur_.pose.rotationMatrix() * forward_direction_body;
  forward_direction_global.normalize();
  
  // 降低前进方向的权重（增大噪声）
  double forward_noise_scale = 10.0; // 前进方向噪声增大5倍

  Eigen::Vector15d last_dx = Eigen::Vector15d::Zero();
  double weight = 1000;
  Eigen::Matrix3d R_inv = Eigen::Matrix3d::Identity() * weight;
  //setupCorridorSpecificWeights(R_inv, forward_direction_global, forward_noise_scale);
  
  int j = 0;
  const auto &cur_pose = bodystate_cur_.pose;

  Eigen::Matrix15d KH;
  for (j = 0; j < liopara_.max_iteration; ++j) {
    Vector3dVector points_w = source;

    TransformPoints(cur_pose.matrix(), points_w);

    const auto &[src, tgt] =
        lio_map_.GetCorrespondences(points_w, max_correspondence_distance);

    auto compute_jacobian_and_residual = [&](auto i) {
      const Eigen::Vector3d &source_pt = src[i];
      const Eigen::Vector3d &target_pt = tgt[i];
      
      const Eigen::Vector3d residual = (source_pt - target_pt);
      Eigen::Vector3d R_bG_p = source_pt - cur_pose.translation();
      Eigen::Matrix3_15d H = Eigen::Matrix3_15d::Zero(); // 3行15列。 3：error维度，15 delta_x维度
      H.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
      H.block<3, 3>(0, 6) = Sophus::SO3d::hat(R_bG_p);
      //H.block<3, 3>(0, 6) = Sophus::SO3d::hat(cur_pose.matrix() * source_pt);

      return std::make_tuple(H, residual);
    };

    const auto [HTRH, HTRz] = tbb::parallel_reduce(
        // Range
        tbb::blocked_range<size_t>{0, src.size()},
        // Identity
        ResultTuple(),
        // 1st Lambda: Parallel computation
        [&](const tbb::blocked_range<size_t> &r, ResultTuple J) -> ResultTuple {
          auto &[HTRH_private, HTRz_private] = J;
          for (auto i = r.begin(); i < r.end(); ++i) {
            const auto &[H, z] = compute_jacobian_and_residual(i);
            HTRH_private.noalias() += H.transpose() * R_inv * H;
            HTRz_private.noalias() += H.transpose() * R_inv * z;
          }
          return J;
        },
        // 2nd Lambda: Parallel reduction of the private Jacboians
        [&](ResultTuple a, const ResultTuple &b) -> ResultTuple {
          return a + b;
        });

    // if(j == 0)
    // {
    //   //checkLidarDegeneration(HTRH);
    // }

    Eigen::Matrix15d S_inv = (HTRH + Cov_.inverse()).inverse();
    delta_x_ = S_inv * HTRz;
    KH = S_inv * HTRH;
    stateFeedback();

    if ((delta_x_ - last_dx).norm() < 0.001) {
      break;
    }
    last_dx = delta_x_;
    delta_x_.setZero();
  }
  Cov_ -= KH * Cov_;

  Sophus::SE3d pose_in_lidar_frame =
      bodystate_cur_.pose * Sophus::SE3d(liopara_.Trans_lidar_imu);
  Imu_Prediction_Covariance_.setZero();
  lio_map_.Update(frame_downsample, pose_in_lidar_frame);
  last_update_t_ = lidar_t_;

  bodystate_pre_lidar_update_ = bodystate_cur_;
}

void LIOEKF::resetCov(Eigen::Matrix15d &Cov) {

  Cov.setZero();
  ImuError imuerror_std = liopara_.initstate_std.imuerror;

  Cov.block(POS_ID, POS_ID, 3, 3) =
      create_covariance_block(liopara_.initstate_std.pos);
  Cov.block(VEL_ID, VEL_ID, 3, 3) =
      create_covariance_block(liopara_.initstate_std.vel);
  Cov.block(ATT_ID, ATT_ID, 3, 3) =
      create_covariance_block(liopara_.initstate_std.euler);
  Cov.block(GYRO_BIAS_ID, GYRO_BIAS_ID, 3, 3) =
      create_covariance_block(imuerror_std.gyrbias);
  Cov.block(ACC_BIAS_ID, ACC_BIAS_ID, 3, 3) =
      create_covariance_block(imuerror_std.accbias);
}

int LIOEKF::isToUpdate(double imutime1, double imutime2,
                       double updatetime) const {

  if (abs(imutime1 - updatetime) < TIME_ALIGN_ERR) {

    // updatetime is near to imutime1
    return 1;
  } else if (abs(imutime2 - updatetime) <= TIME_ALIGN_ERR) {

    // updatetime is near to imutime2
    return 2;
  } else if (imutime1 < updatetime && updatetime < imutime2) {

    // updatetime is between imutime1 and imutime2, but not near to either
    return 3;
  } else {

    // updatetime is not bewteen imutime1 and imutime2, and not near to either.
    return 0;
  }
}

void LIOEKF::ekfPredict(Eigen::Matrix15d &State_Transition_Mat,
                        Eigen::Matrix15d &Process_Noise_Cov) {

  assert(State_Transition_Mat.rows() == Cov_.rows());
  assert(Process_Noise_Cov.rows() == Cov_.rows());
  auto propagate = [&](const Eigen::Matrix15d &Sigma) {
    return State_Transition_Mat * Sigma * State_Transition_Mat.transpose() +
           Process_Noise_Cov;
  };
  // propagate system covariance and error state
  Cov_ = State_Transition_Mat * Cov_ * State_Transition_Mat.transpose() +
         Process_Noise_Cov;
  Imu_Prediction_Covariance_ =
      State_Transition_Mat * Imu_Prediction_Covariance_ *
          State_Transition_Mat.transpose() +
      Process_Noise_Cov + 1e-8 * Eigen::Matrix15d::Identity();
  delta_x_ = State_Transition_Mat * delta_x_;
}

void LIOEKF::ekfUpdate(Eigen::MatrixXd &dz, Eigen::MatrixXd &H,
                       Eigen::MatrixXd &R) {
  // debug_<<"ekfUpdate start"<<std::endl;
  assert(H.cols() == Cov_.rows());
  assert(dz.rows() == H.rows());
  assert(dz.rows() == R.rows());
  assert(dz.cols() == 1);

  // Compute Kalman Gain
  auto temp = H * Cov_ * H.transpose() + R;
  Eigen::MatrixXd K = Cov_ * H.transpose() * temp.inverse();

  // update system error state and covariance
  Eigen::Matrix15d I;
  I.setIdentity();
  I = I - K * H;

  delta_x_ = delta_x_ + K * (dz - H * delta_x_);
  Cov_ = I * Cov_ * I.transpose() + K * R * K.transpose();
}

void LIOEKF::stateFeedback() {
  // position error feedback
  Eigen::Vector3d delta_translation = delta_x_.block(POS_ID, 0, 3, 1);
  Eigen::Vector3d delta_quat = delta_x_.block(ATT_ID, 0, 3, 1);
  bodystate_cur_.pose.so3() =
      Sophus::SO3d::exp(delta_quat) * bodystate_cur_.pose.so3();
  bodystate_cur_.pose.translation() =
      bodystate_cur_.pose.translation() - delta_translation;
  // velocity error feedback
  Eigen::Vector3d delta_vel = delta_x_.block(VEL_ID, 0, 3, 1);
  bodystate_cur_.vel -= delta_vel;
  // IMU bias error feedback
  Eigen::Vector3d delta_bias_gyro = delta_x_.block(GYRO_BIAS_ID, 0, 3, 1);
  imuerror_.gyrbias += delta_bias_gyro;
  Eigen::Vector3d delta_bias_acc = delta_x_.block(ACC_BIAS_ID, 0, 3, 1);
  imuerror_.accbias += delta_bias_acc;

}

NavState LIOEKF::getNavState() {
  NavState state;
  state.pos = bodystate_cur_.pose.translation();
  state.vel = bodystate_cur_.vel;
  state.euler = Rotation::matrix2euler(bodystate_cur_.pose.rotationMatrix());
  state.imuerror = imuerror_;
  return state;
}

std::pair<Vector3dVector, Vector3dVector>
LIOEKF::Voxelize(const std::vector<Eigen::Vector3d> &frame) const {
  const auto voxel_size = liopara_.voxel_size;
  const auto frame_downsample =
      kiss_icp::VoxelDownsample(frame, voxel_size * 0.5);
  const auto source =
      kiss_icp::VoxelDownsample(frame_downsample, voxel_size * 1.5);
  return {source, frame_downsample};
}

Eigen::Matrix4d LIOEKF::poseTran(const Eigen::Matrix4d pose1,
                                 const Eigen::Matrix4d pose2) {
  Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
  Eigen::Matrix3d R1, R2;
  Eigen::Vector3d T1, T2;

  R1 = pose1.block<3, 3>(0, 0);
  T1 = pose1.block<3, 1>(0, 3);
  R2 = pose2.block<3, 3>(0, 0);
  T2 = pose2.block<3, 1>(0, 3);

  pose.block<3, 3>(0, 0) = R1 * R2;
  pose.block<3, 1>(0, 3) = T1 + R1 * T2;

  return pose;
}

} // namespace lio_ekf

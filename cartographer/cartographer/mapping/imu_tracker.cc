/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "cartographer/mapping/imu_tracker.h"

#include <cmath>
#include <limits>

#include "cartographer/common/math.h"
#include "cartographer/transform/transform.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {

ImuTracker::ImuTracker(const double imu_gravity_time_constant,
                       const common::Time time)
    : imu_gravity_time_constant_(imu_gravity_time_constant),
      time_(time),
      last_linear_acceleration_time_(common::Time::min()),
      orientation_(Eigen::Quaterniond::Identity()),
      gravity_vector_(Eigen::Vector3d::UnitZ()),
      imu_angular_velocity_(Eigen::Vector3d::Zero()) {}

//(cxn)使用角速度 更新旋转，以及重力向量
// gravity_vector_为重力向量在baselink系中坐标
void ImuTracker::Advance(const common::Time time) {
  CHECK_LE(time_, time);
  const double delta_t = common::ToSeconds(time - time_);
  const Eigen::Quaterniond rotation =
      transform::AngleAxisVectorToRotationQuaternion(
          Eigen::Vector3d(imu_angular_velocity_ * delta_t));
  orientation_ = (orientation_ * rotation).normalized();
  gravity_vector_ = rotation.conjugate() * gravity_vector_;
  time_ = time;
}

//(cxn)使用加速度更新 重力向量 方向，以此更新旋转
void ImuTracker::AddImuLinearAccelerationObservation(
    const Eigen::Vector3d& imu_linear_acceleration) {
  // Update the 'gravity_vector_' with an exponential moving average using the
  // 'imu_gravity_time_constant'.
  const double delta_t =
      last_linear_acceleration_time_ > common::Time::min()
          ? common::ToSeconds(time_ - last_linear_acceleration_time_)
          : std::numeric_limits<double>::infinity();
  last_linear_acceleration_time_ = time_;
  //(cxn)?搞不懂这里为啥这么设置
  //滑动平均 g = (1-alpha)*g+ alpha*linear_acc;
  const double alpha = 1. - std::exp(-delta_t / imu_gravity_time_constant_);
  gravity_vector_ =
      (1. - alpha) * gravity_vector_ + alpha * imu_linear_acceleration;
  // Change the 'orientation_' so that it agrees with the current
  // 'gravity_vector_'.
  //(cxn) Eigen::Quaterniond::FromTwoVectors(Pa,Pb)=Rba
  // 举个例子：二维旋转 R=[sqrt(2)/2 -sqrt(2)/2; sqrt(2)/2 sqrt(2)/2]
  // inv(R)*[0,1]=[sqrt(2)/2 sqrt(2)/2]，意为，车在绕z轴了45度，重力方向在车坐标系中向量为 Zimu=[sqrt(2)/2 sqrt(2)/2]
  // 但是加速度计 测重力更准，经过 指数平滑平均，测出 为 Zacc=[sqrt(3)/2 1/2]
  // 我们计算出 Rimu-acc 即为向量Zacc到向量Zimu，在本例中，就是绕z轴15度
  // 所以，我们调整 二维旋转为绕z轴 45+15=60度 
  const Eigen::Quaterniond rotation = Eigen::Quaterniond::FromTwoVectors(
      gravity_vector_, orientation_.conjugate() * Eigen::Vector3d::UnitZ());
  orientation_ = (orientation_ * rotation).normalized();
  //数学上来讲有 orientation*gravityvector==[0,0,1]
  CHECK_GT((orientation_ * gravity_vector_).z(), 0.);
  CHECK_GT((orientation_ * gravity_vector_).normalized().z(), 0.99);
}

//(cxn)在没有IMU数据之前，是由外推器或里程计核算出的角速度更新
// 有了IMU数据之后，就用IMU的角速度
void ImuTracker::AddImuAngularVelocityObservation(
    const Eigen::Vector3d& imu_angular_velocity) {
  imu_angular_velocity_ = imu_angular_velocity;
}

}  // namespace mapping
}  // namespace cartographer

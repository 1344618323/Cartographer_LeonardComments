/*
 * Copyright 2017 The Cartographer Authors
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

#include "cartographer/mapping/pose_extrapolator.h"

#include <algorithm>

#include "cartographer/common/make_unique.h"
#include "cartographer/transform/transform.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {

/*(cxn)
这个类其实就是用来推算机器人在local_frame中的位姿的
local_frame在cartographer中的意思就是未经过回环检测与pose-graph优化的坐标系

核心函数 ExtrapolatePose(time)就是推算 time 时刻机器人位姿的

每次local—trajectory-bulider中经过scan-match优化后，会将优化后的位姿（3D的）加入到timed_pose_queue_中（即local—trajectory-bulider 调用函数 addpose）
timed_pose_queue_.back() 就是 最新的优化结果，我们记为 Tlw-t1

然后 ExtrapolatePose(time) 用来 在 Tlw-t1的基础上计算 Tlw-t2

每次 Tlw-t1 加入之后，会计算 (Tlw-t0'*Tlw-t1) ->  线速度 V = t(t1)-t(t0)/(t1-t0)  角速度 w = 转成旋转向量(Rlw-t0'*Rlw-t1) /(t1-t0)
odom也会算 线速度与角速度，待会再说

同时这里有个对象，Imu-tracker-用来一直估计 机器人的旋转， 我们可以认为这个传感器的坐标系为 imu-frame，其估计的旋转值为 Rimuinit-t吧
在没有IMU数据的时候，就用上面推算的角速度w 模拟 真实传感器IMU的输入，同时用 (1,0,0)*10 模拟真实传感器加速度计的数值
Imu-tracker-的两种更新情况：   
    A. 每次 local—trajectory-bulider 调用函数 addpose时更新
    B. 每次 local—trajectory-bulider 调用函数 EstimateGravityOrientation 时更新 （就是搜集好雷达数据之后，准备scan-match优化之前）

可以发现在local—trajectory-bulider-2d中，cartographer是十分信任 Imu-tracker-的旋转的（除了yaw），
每次scan-match优化后，会将优化输出的2D变换矩阵T2d 操作以 {F2d->3d(T2d)}*R(imu)，作为加入到timed_pose_queue_中位姿
还不知道3d中是怎么搞的


说下ExtrapolatePose(time)怎么做的吧：
已知Tlw-t1=[R(t1) t(t1)]
要算Tlw-t1=[R(t2) t(t2)]

1. t(t2)= t(t1) + v(t2-t1)
2. 使用对象 extrapolation-imu-tracker- 来推算角度
    extrapolation-imu-tracker- 在每次 Imu-tracker-更新后再复制，
    这样用 extrapolation-imu-tracker-就能先更新到时刻t2，而不影响Imu-tracker-保持在t1的状态了
    此时，extrapolation-imu-tracker-算出的旋转为 Rimuinit-t2，而Imu-tracker-的状态为Rimuinit-t1

    R(t2)= R(t1)* (Rimuinit-t1)'*(Rimuinit-t2)


最后说一下odom是怎么估计线速度与角速度的吧
已知在 odom-frame 坐标系下，有one、two两个时刻的位姿数据：Todom-one，Todom-two，two是最新的时刻
时刻差为 deltat=one-two
有 Todom-two'*Todom-one=[ R(two)'R(one)  R(two)'(t(one)-t(two))]
1. 角速度无疑问就是 w = 转成旋转向量(R(two)'R(one)) /deltat
2. 线速度有个问题就是要 求位移差(t(one)-t(two))  在 lcaol-frame中的坐标，
   (t(one)-t(two))是odom-frame中坐标，应写成 t(odom-one)-t(odom-two)
    使用对象 odometry-imu-tracker- 来推算角度
    odometry-imu-tracker- 也是在每次 Imu-tracker-更新后再复制，与extrapolation-imu-tracker-同理， 
    two时刻，odometry-imu-tracker-算出的旋转为 Rimuinit-two，而Imu-tracker-的状态为Rimuinit-t1
    可以推算出机器人在two时刻时，在local-frame中的旋转：     Rlw-two= Rlw-t1* (Rimuinit-t1)'*(Rimuinit-two)

  local-frame到 odom-frame的变换为 Tlw-odom,其中tlw-dom的值我们不用管，因为 (t(one)-t(two))是个差，变换后关于 tlw-dom 的部分，自然会被消掉
  也就说  Tlw-odom*(t(odom-one)-t(odom-two))= Rlw-odom*t(odom-one)-t(odom-two)
  Rlw-odom= Rlw-two* R(two)'
  所以，线速度为 {Rlw-two* R(two)'*(t(one)-t(two))}/deltat

*/

PoseExtrapolator::PoseExtrapolator(const common::Duration pose_queue_duration,
                                   double imu_gravity_time_constant)
    : pose_queue_duration_(pose_queue_duration),
      gravity_time_constant_(imu_gravity_time_constant),
      cached_extrapolated_pose_{common::Time::min(),
                                transform::Rigid3d::Identity()} {}

std::unique_ptr<PoseExtrapolator> PoseExtrapolator::InitializeWithImu(
    const common::Duration pose_queue_duration,
    const double imu_gravity_time_constant, const sensor::ImuData& imu_data) {
  auto extrapolator = common::make_unique<PoseExtrapolator>(
      pose_queue_duration, imu_gravity_time_constant);
  extrapolator->AddImuData(imu_data);
  extrapolator->imu_tracker_ =
      common::make_unique<ImuTracker>(imu_gravity_time_constant, imu_data.time);
  extrapolator->imu_tracker_->AddImuLinearAccelerationObservation(
      imu_data.linear_acceleration);
  extrapolator->imu_tracker_->AddImuAngularVelocityObservation(
      imu_data.angular_velocity);
  extrapolator->imu_tracker_->Advance(imu_data.time);
  extrapolator->AddPose(
      imu_data.time,
      transform::Rigid3d::Rotation(extrapolator->imu_tracker_->orientation()));
  return extrapolator;
}

common::Time PoseExtrapolator::GetLastPoseTime() const {
  if (timed_pose_queue_.empty()) {
    return common::Time::min();
  }
  return timed_pose_queue_.back().time;
}

common::Time PoseExtrapolator::GetLastExtrapolatedTime() const {
  if (!extrapolation_imu_tracker_) {
    return common::Time::min();
  }
  return extrapolation_imu_tracker_->time();
}

void PoseExtrapolator::AddPose(const common::Time time,
                               const transform::Rigid3d& pose) {
  if (imu_tracker_ == nullptr) {
    common::Time tracker_start = time;
    if (!imu_data_.empty()) {
      tracker_start = std::min(tracker_start, imu_data_.front().time);
    }
    imu_tracker_ =
        common::make_unique<ImuTracker>(gravity_time_constant_, tracker_start);
  }
  timed_pose_queue_.push_back(TimedPose{time, pose});
  while (timed_pose_queue_.size() > 2 &&
         timed_pose_queue_[1].time <= time - pose_queue_duration_) {
           //(cxn)我们还是希望timedposequeue的第一个值与最后一个值的时间差较大的，否则算出来的速度不太好
           //当[1]值与新值的时间差在0.001s以上，我们才把第一个值去了
    timed_pose_queue_.pop_front();
  }
  UpdateVelocitiesFromPoses();
  AdvanceImuTracker(time, imu_tracker_.get());
  TrimImuData();
  TrimOdometryData();
  //(cxn)这个变量只在推算 odom线速度、角速度时才有用到，注意用了拷贝构造函数
  odometry_imu_tracker_ = common::make_unique<ImuTracker>(*imu_tracker_);
  // 外部调用函数ExtrapolatePose() 会用到这个变量
  extrapolation_imu_tracker_ = common::make_unique<ImuTracker>(*imu_tracker_);
  //可以发现imu_tracker_与timedposequeue没半毛钱关系，可以看做是对IMU数据用于获取角度的处理吧，
  //而odometry_imu_tracker_与extrapolation_imu_tracker_都是在对timedposequeue扩展时用到的
}

void PoseExtrapolator::AddImuData(const sensor::ImuData& imu_data) {
  CHECK(timed_pose_queue_.empty() ||
        imu_data.time >= timed_pose_queue_.back().time);
  imu_data_.push_back(imu_data);
  TrimImuData();
}

void PoseExtrapolator::AddOdometryData(
    const sensor::OdometryData& odometry_data) {
  CHECK(timed_pose_queue_.empty() ||
        odometry_data.time >= timed_pose_queue_.back().time);
  odometry_data_.push_back(odometry_data);
  TrimOdometryData();
  if (odometry_data_.size() < 2) {
    return;
  }
  // TODO(whess): Improve by using more than just the last two odometry poses.
  // Compute extrapolation in the tracking frame.
  const sensor::OdometryData& odometry_data_oldest = odometry_data_.front();
  const sensor::OdometryData& odometry_data_newest = odometry_data_.back();
  const double odometry_time_delta =
      common::ToSeconds(odometry_data_oldest.time - odometry_data_newest.time);
  //(cxn)求 deltaT = T2‘*T1=[R2'R1,R2'(t1-t2);0 1]
  const transform::Rigid3d odometry_pose_delta =
      odometry_data_newest.pose.inverse() * odometry_data_oldest.pose;
  // =R2'R1/delta(t)
  angular_velocity_from_odometry_ =
      transform::RotationQuaternionToAngleAxisVector(
          odometry_pose_delta.rotation()) /
      odometry_time_delta;
  if (timed_pose_queue_.empty()) {
    return;
  }
  // 使用IMU推测出此时的外推器的旋转 Rpose
  // 这里要将里程计坐标系下的位移转换成外推器坐标系pose下的位移
  // 从A点到B点
  // Rpose-B * Rodom-B' *(todom-A - todom-B) /delta(t)
  // = Rpose-odom * (todom-A - todom-B) /delta(t)
  // = (tpose-A - tpose-B) /delta(t)
  const Eigen::Vector3d
      linear_velocity_in_tracking_frame_at_newest_odometry_time =
          odometry_pose_delta.translation() / odometry_time_delta;
  const Eigen::Quaterniond orientation_at_newest_odometry_time =
      timed_pose_queue_.back().pose.rotation() *
      ExtrapolateRotation(odometry_data_newest.time,
                          odometry_imu_tracker_.get());
  linear_velocity_from_odometry_ =
      orientation_at_newest_odometry_time *
      linear_velocity_in_tracking_frame_at_newest_odometry_time;
}

transform::Rigid3d PoseExtrapolator::ExtrapolatePose(const common::Time time) {
  const TimedPose& newest_timed_pose = timed_pose_queue_.back();
  CHECK_GE(time, newest_timed_pose.time);
  if (cached_extrapolated_pose_.time != time) {
    //(cxn)在timedposequeue的基础上匀速推断位移
    const Eigen::Vector3d translation =
        ExtrapolateTranslation(time) + newest_timed_pose.pose.translation();
    //(cxn)在timedposequeue的基础上用imuTracker推算旋转
    const Eigen::Quaterniond rotation =
        newest_timed_pose.pose.rotation() *
        ExtrapolateRotation(time, extrapolation_imu_tracker_.get());
    cached_extrapolated_pose_ =
        TimedPose{time, transform::Rigid3d{translation, rotation}};
  }
  return cached_extrapolated_pose_.pose;
}

//(cxn)更新imu_tracker_
Eigen::Quaterniond PoseExtrapolator::EstimateGravityOrientation(
    const common::Time time) {
  ImuTracker imu_tracker = *imu_tracker_;
  AdvanceImuTracker(time, &imu_tracker);
  return imu_tracker.orientation();
}

//(cxn)使用外推器计算出的位姿 计算 线速度、角速度
//怎么算？
// a为之前的位姿Twa，b为新的位姿Twb， Tab=[Ra'Rb,Ra'(tb-ta);0,1]
// deltaA是a的时间戳，deltaB是b的时间戳
// 角速度就是  Ra'Rb/（deltaB-deltaA）
// 线速度是  （tb-ta）/（deltaB-deltaA）
void PoseExtrapolator::UpdateVelocitiesFromPoses() {
  if (timed_pose_queue_.size() < 2) {
    // We need two poses to estimate velocities.
    return;
  }
  CHECK(!timed_pose_queue_.empty());
  const TimedPose& newest_timed_pose = timed_pose_queue_.back();
  const auto newest_time = newest_timed_pose.time;
  const TimedPose& oldest_timed_pose = timed_pose_queue_.front();
  const auto oldest_time = oldest_timed_pose.time;
  const double queue_delta = common::ToSeconds(newest_time - oldest_time);
  if (queue_delta < 0.001) {  // 1 ms
    LOG(WARNING) << "Queue too short for velocity estimation. Queue duration: "
                 << queue_delta << " ms";
    return;
  }
  const transform::Rigid3d& newest_pose = newest_timed_pose.pose;
  const transform::Rigid3d& oldest_pose = oldest_timed_pose.pose;
  linear_velocity_from_poses_ =
      (newest_pose.translation() - oldest_pose.translation()) / queue_delta;
  angular_velocity_from_poses_ =
      transform::RotationQuaternionToAngleAxisVector(
          oldest_pose.rotation().inverse() * newest_pose.rotation()) /
      queue_delta;
}

//(cxn)如果[1]数据的时间比timedposequeue最后一帧的时间还提前，就把[0]清了（循环）
void PoseExtrapolator::TrimImuData() {
  while (imu_data_.size() > 1 && !timed_pose_queue_.empty() &&
         imu_data_[1].time <= timed_pose_queue_.back().time) {
    imu_data_.pop_front();
  }
}

void PoseExtrapolator::TrimOdometryData() {
  while (odometry_data_.size() > 2 && !timed_pose_queue_.empty() &&
         odometry_data_[1].time <= timed_pose_queue_.back().time) {
    odometry_data_.pop_front();
  }
}

//(cxn)用于更新IMU追踪器，详见imu_tracker.h中注释
void PoseExtrapolator::AdvanceImuTracker(const common::Time time,
                                         ImuTracker* const imu_tracker) const {
  CHECK_GE(time, imu_tracker->time());
  if (imu_data_.empty() || time < imu_data_.front().time) {
    // There is no IMU data until 'time', so we advance the ImuTracker and use
    // the angular velocities from poses and fake gravity to help 2D stability.
    imu_tracker->Advance(time);
    imu_tracker->AddImuLinearAccelerationObservation(Eigen::Vector3d::UnitZ());
    imu_tracker->AddImuAngularVelocityObservation(
        odometry_data_.size() < 2 ? angular_velocity_from_poses_
                                  : angular_velocity_from_odometry_);
    return;
  }
  //(cxn)可以发现在这一个PoseExtapolator对象中存在3个imuTracker对象
  if (imu_tracker->time() < imu_data_.front().time) {
    // Advance to the beginning of 'imu_data_'.
    imu_tracker->Advance(imu_data_.front().time);
  }
  auto it = std::lower_bound(
      imu_data_.begin(), imu_data_.end(), imu_tracker->time(),
      [](const sensor::ImuData& imu_data, const common::Time& time) {
        return imu_data.time < time;
      });
  while (it != imu_data_.end() && it->time < time) {
    imu_tracker->Advance(it->time);
    imu_tracker->AddImuLinearAccelerationObservation(it->linear_acceleration);
    imu_tracker->AddImuAngularVelocityObservation(it->angular_velocity);
    ++it;
  }
  imu_tracker->Advance(time);
}

//(cxn)推进参数imu_tracker更新到time，返回与类变量imu_tracker_的旋转差值
//我们标记成 inv(Rimu_lt)*Rthis_imu_t，相当于是获得了相对于原来imu_tracker_的角度变化
//其实就是计算 机器人 从 位姿 A 到 位姿 B 的旋转矩阵 RA‘RB
Eigen::Quaterniond PoseExtrapolator::ExtrapolateRotation(
    const common::Time time, ImuTracker* const imu_tracker) const {
  CHECK_GE(time, imu_tracker->time());
  AdvanceImuTracker(time, imu_tracker);
  const Eigen::Quaterniond last_orientation = imu_tracker_->orientation();
  return last_orientation.inverse() * imu_tracker->orientation();
}

//(cxn)推算时刻time的位移要用这个函数
// 即(time-timedposequeue)*v；v由里程计或者外推器获得
Eigen::Vector3d PoseExtrapolator::ExtrapolateTranslation(common::Time time) {
  const TimedPose& newest_timed_pose = timed_pose_queue_.back();
  const double extrapolation_delta =
      common::ToSeconds(time - newest_timed_pose.time);
  if (odometry_data_.size() < 2) {
    return extrapolation_delta * linear_velocity_from_poses_;
  }
  return extrapolation_delta * linear_velocity_from_odometry_;
}

}  // namespace mapping
}  // namespace cartographer

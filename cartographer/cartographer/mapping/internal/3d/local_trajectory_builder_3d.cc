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

#include "cartographer/mapping/internal/3d/local_trajectory_builder_3d.h"

#include <memory>

#include "cartographer/common/make_unique.h"
#include "cartographer/common/time.h"
#include "cartographer/mapping/internal/3d/scan_matching/rotational_scan_matcher.h"
#include "cartographer/mapping/proto/3d/local_trajectory_builder_options_3d.pb.h"
#include "cartographer/mapping/proto/3d/submaps_options_3d.pb.h"
#include "cartographer/mapping/proto/scan_matching//ceres_scan_matcher_options_3d.pb.h"
#include "cartographer/mapping/proto/scan_matching//real_time_correlative_scan_matcher_options.pb.h"
#include "glog/logging.h"

/*cxn
local-3d-slam 是 local-2d-slam 的延伸，写下不同的地方

* correlative-scan-match:
  2d中为了加速，先对点云做二维旋转，得到 r 组点云，再分别平移，得到 r*x*y组 点云，然后分别算得分
    先旋转的原因在于只需要调用r×n次 sin、cos函数（n为点云个数）；若先平移再旋转则要调用 x*y*r*n次 sin、cos函数
  3d中就纯粹暴力地算出 x*y*z*rx*ry*rz 种刚体变换矩阵，然后分别乘到点云上算得分，（注意旋转 用的是 旋转向量 ，而不是sin、cos）
    代码实现中，估计作者觉得也没用到三角函数，纯粹都是线性代数，就没必要向2d那样调整顺序了。

* submap3d的实现
  3d由两张不同分辨率的hybridMap组成：高分辨率地图（如0.1m），低分辨率地图（如0.45m）
    其中，高分辨率地图不会管距离比较远的点（如20m外的点）
    hybridMap实现了3d栅格地图，为了节约内存，只会对用到的cell申请内存（类似八叉树），详见 bybrid_grid.h
  2d中新建submap在localframe中的位姿为 (新建submap时机器人在localframe中的 x，y; 旋转角度设置为0)，2维地图的平面只要与水平面平行就是了
  3d中新建submap的位姿：平移量依然是 (机器人在localframe中的 x，y，z)，而旋转是IMU测出的旋转（机器人在IMU坐标系下的旋转）

* ceres-scan-match
  与2d思想一致，基于非线性优化的扫描匹配，因此要对栅格地图做插值才能求导
  作者提到，由于ceres-scan-match的性能原因，才给submap3d加了高低分辨率的设定
  且在默认配置文件中，costFunction中低分辨率地图的权值比高分辨率地图的权值高
  我的理解是：3d中，优化非常消耗时间，低分辨率地图 较高的权值能让优化算法较快地迭代出 一个较好的位姿，然后高分辨率地图再去进一步优化
*/

namespace cartographer {
namespace mapping {

static auto* kLocalSlamLatencyMetric = metrics::Gauge::Null();
static auto* kRealTimeCorrelativeScanMatcherScoreMetric =
    metrics::Histogram::Null();
static auto* kCeresScanMatcherCostMetric = metrics::Histogram::Null();
static auto* kScanMatcherResidualDistanceMetric = metrics::Histogram::Null();
static auto* kScanMatcherResidualAngleMetric = metrics::Histogram::Null();

LocalTrajectoryBuilder3D::LocalTrajectoryBuilder3D(
    const mapping::proto::LocalTrajectoryBuilderOptions3D& options,
    const std::vector<std::string>& expected_range_sensor_ids)
    : options_(options),
      active_submaps_(options.submaps_options()),
      motion_filter_(options.motion_filter_options()),
      real_time_correlative_scan_matcher_(
          common::make_unique<scan_matching::RealTimeCorrelativeScanMatcher3D>(
              options_.real_time_correlative_scan_matcher_options())),
      ceres_scan_matcher_(
          common::make_unique<scan_matching::CeresScanMatcher3D>(
              options_.ceres_scan_matcher_options())),
      accumulated_range_data_{Eigen::Vector3f::Zero(), {}, {}},
      range_data_collator_(expected_range_sensor_ids) {}

LocalTrajectoryBuilder3D::~LocalTrajectoryBuilder3D() {}

void LocalTrajectoryBuilder3D::AddImuData(const sensor::ImuData& imu_data) {
  if (extrapolator_ != nullptr) {
    extrapolator_->AddImuData(imu_data);
    return;
  }
  // We derive velocities from poses which are at least 1 ms apart for numerical
  // stability. Usually poses known to the extrapolator will be further apart
  // in time and thus the last two are used.
  constexpr double kExtrapolationEstimationTimeSec = 0.001;
  extrapolator_ = mapping::PoseExtrapolator::InitializeWithImu(
      ::cartographer::common::FromSeconds(kExtrapolationEstimationTimeSec),
      options_.imu_gravity_time_constant(), imu_data);
}

std::unique_ptr<LocalTrajectoryBuilder3D::MatchingResult>
LocalTrajectoryBuilder3D::AddRangeData(
    const std::string& sensor_id,
    const sensor::TimedPointCloudData& unsynchronized_data) {
  auto synchronized_data =
      range_data_collator_.AddRangeData(sensor_id, unsynchronized_data);
  if (synchronized_data.ranges.empty()) {
    LOG(INFO) << "Range data collator filling buffer.";
    return nullptr;
  }

  const common::Time& time = synchronized_data.time;
  if (extrapolator_ == nullptr) {
    // Until we've initialized the extrapolator with our first IMU message, we
    // cannot compute the orientation of the rangefinder.
    LOG(INFO) << "IMU not yet initialized.";
    return nullptr;
  }

  CHECK(!synchronized_data.ranges.empty());
  CHECK_LE(synchronized_data.ranges.back().point_time[3], 0.f);
  const common::Time time_first_point =
      time +
      common::FromSeconds(synchronized_data.ranges.front().point_time[3]);
  if (time_first_point < extrapolator_->GetLastPoseTime()) {
    LOG(INFO) << "Extrapolator is still initializing.";
    return nullptr;
  }

  if (num_accumulated_ == 0) {
    accumulation_started_ = std::chrono::steady_clock::now();
  }

  std::vector<sensor::TimedPointCloudOriginData::RangeMeasurement> hits =
      sensor::VoxelFilter(0.5f * options_.voxel_filter_size())
          .Filter(synchronized_data.ranges);

  std::vector<transform::Rigid3f> hits_poses;
  hits_poses.reserve(hits.size());
  bool warned = false;
  for (const auto& hit : hits) {
    common::Time time_point = time + common::FromSeconds(hit.point_time[3]);
    if (time_point < extrapolator_->GetLastExtrapolatedTime()) {
      if (!warned) {
        LOG(ERROR)
            << "Timestamp of individual range data point jumps backwards from "
            << extrapolator_->GetLastExtrapolatedTime() << " to " << time_point;
        warned = true;
      }
      time_point = extrapolator_->GetLastExtrapolatedTime();
    }
    hits_poses.push_back(
        extrapolator_->ExtrapolatePose(time_point).cast<float>());
  }

  if (num_accumulated_ == 0) {
    // 'accumulated_range_data_.origin' is not used.
    accumulated_range_data_ = sensor::RangeData{{}, {}, {}};
  }

  for (size_t i = 0; i < hits.size(); ++i) {
    const Eigen::Vector3f hit_in_local =
        hits_poses[i] * hits[i].point_time.head<3>();
    const Eigen::Vector3f origin_in_local =
        hits_poses[i] * synchronized_data.origins.at(hits[i].origin_index);
    const Eigen::Vector3f delta = hit_in_local - origin_in_local;
    const float range = delta.norm();
    if (range >= options_.min_range()) {
      if (range <= options_.max_range()) {
        accumulated_range_data_.returns.push_back(hit_in_local);
      } else {
        // We insert a ray cropped to 'max_range' as a miss for hits beyond the
        // maximum range. This way the free space up to the maximum range will
        // be updated.
        accumulated_range_data_.misses.push_back(
            origin_in_local + options_.max_range() / range * delta);
      }
    }
  }
  ++num_accumulated_;

  //3d.lua中 numaccumulatedrangedata=160
  if (num_accumulated_ >= options_.num_accumulated_range_data()) {
    num_accumulated_ = 0;
    transform::Rigid3f current_pose =
        extrapolator_->ExtrapolatePose(time).cast<float>();
    const sensor::RangeData filtered_range_data = {
        current_pose.translation(),
        sensor::VoxelFilter(options_.voxel_filter_size())
            .Filter(accumulated_range_data_.returns),
        sensor::VoxelFilter(options_.voxel_filter_size())
            .Filter(accumulated_range_data_.misses)};
    return AddAccumulatedRangeData(
        //(cxn)该函数中第二个参数是去畸变后的点云数据（RangeData对象），这些RangeData对象的origin都是(0,0,0): [R' -R't; 0 1]*[t;1]=[0;0;0]
        time, sensor::TransformRangeData(filtered_range_data,
                                         current_pose.inverse()));
  }
  return nullptr;
}

std::unique_ptr<LocalTrajectoryBuilder3D::MatchingResult>
LocalTrajectoryBuilder3D::AddAccumulatedRangeData(
    const common::Time time,
    const sensor::RangeData& filtered_range_data_in_tracking) {
  if (filtered_range_data_in_tracking.returns.empty()) {
    LOG(WARNING) << "Dropped empty range data.";
    return nullptr;
  }

  const transform::Rigid3d pose_prediction =
      extrapolator_->ExtrapolatePose(time);

  // (cxn)可以注意到，submap2d中，是将点云直接以local_frame中的坐标（确切地说还有旋转） 插入到submap中的
  // 而在submap3d中，是将点云以 在submap坐标系中坐标 插入到 submap中的，所以求匹配位姿初值时，要左乘submap位姿的逆
  std::shared_ptr<const mapping::Submap3D> matching_submap =
      active_submaps_.submaps().front();
  transform::Rigid3d initial_ceres_pose =
      matching_submap->local_pose().inverse() * pose_prediction;
  sensor::AdaptiveVoxelFilter adaptive_voxel_filter(
      options_.high_resolution_adaptive_voxel_filter_options());
  const sensor::PointCloud high_resolution_point_cloud_in_tracking =
      adaptive_voxel_filter.Filter(filtered_range_data_in_tracking.returns);
  if (high_resolution_point_cloud_in_tracking.empty()) {
    LOG(WARNING) << "Dropped empty high resolution point cloud data.";
    return nullptr;
  }
  
  // (cxn)调用correlative-scan-match
  if (options_.use_online_correlative_scan_matching()) {
    // We take a copy since we use 'initial_ceres_pose' as an output argument.
    const transform::Rigid3d initial_pose = initial_ceres_pose;
    // 可以发现只用高分率网格来做 correlative-scan-math
    double score = real_time_correlative_scan_matcher_->Match(
        initial_pose, high_resolution_point_cloud_in_tracking,
        matching_submap->high_resolution_hybrid_grid(), &initial_ceres_pose);
    kRealTimeCorrelativeScanMatcherScoreMetric->Observe(score);
  }

  transform::Rigid3d pose_observation_in_submap;
  ceres::Solver::Summary summary;

  sensor::AdaptiveVoxelFilter low_resolution_adaptive_voxel_filter(
      options_.low_resolution_adaptive_voxel_filter_options());
  const sensor::PointCloud low_resolution_point_cloud_in_tracking =
      low_resolution_adaptive_voxel_filter.Filter(
          filtered_range_data_in_tracking.returns);
  if (low_resolution_point_cloud_in_tracking.empty()) {
    LOG(WARNING) << "Dropped empty low resolution point cloud data.";
    return nullptr;
  }

  // (cxn)调用 ceres-scan-match，高、低分率的地图都会用于优化
  ceres_scan_matcher_->Match(
      (matching_submap->local_pose().inverse() * pose_prediction).translation(),
      initial_ceres_pose,
      {{&high_resolution_point_cloud_in_tracking,
        &matching_submap->high_resolution_hybrid_grid()},
       {&low_resolution_point_cloud_in_tracking,
        &matching_submap->low_resolution_hybrid_grid()}},
      &pose_observation_in_submap, &summary);

  kCeresScanMatcherCostMetric->Observe(summary.final_cost);
  double residual_distance = (pose_observation_in_submap.translation() -
                              initial_ceres_pose.translation())
                                 .norm();
  kScanMatcherResidualDistanceMetric->Observe(residual_distance);
  double residual_angle = pose_observation_in_submap.rotation().angularDistance(
      initial_ceres_pose.rotation());
  kScanMatcherResidualAngleMetric->Observe(residual_angle);

  const transform::Rigid3d pose_estimate =
      matching_submap->local_pose() * pose_observation_in_submap;
  extrapolator_->AddPose(time, pose_estimate);
  const Eigen::Quaterniond gravity_alignment =
      extrapolator_->EstimateGravityOrientation(time);
  
  // 计算点云在localframe中的坐标 plocal = Tlocal-baselink*pbaselink    
  sensor::RangeData filtered_range_data_in_local = sensor::TransformRangeData(
      filtered_range_data_in_tracking, pose_estimate.cast<float>());

  // 插点云，与2d一样，要经过motion-filter才能插点云到submap
  std::unique_ptr<InsertionResult> insertion_result = InsertIntoSubmap(
      time, filtered_range_data_in_local, filtered_range_data_in_tracking,
      high_resolution_point_cloud_in_tracking,
      low_resolution_point_cloud_in_tracking, pose_estimate, gravity_alignment);
  
  auto duration = std::chrono::steady_clock::now() - accumulation_started_;
  kLocalSlamLatencyMetric->Set(
      std::chrono::duration_cast<std::chrono::seconds>(duration).count());
  return common::make_unique<MatchingResult>(MatchingResult{
      time, pose_estimate, std::move(filtered_range_data_in_local),
      std::move(insertion_result)});
}

void LocalTrajectoryBuilder3D::AddOdometryData(
    const sensor::OdometryData& odometry_data) {
  if (extrapolator_ == nullptr) {
    // Until we've initialized the extrapolator we cannot add odometry data.
    LOG(INFO) << "Extrapolator not yet initialized.";
    return;
  }
  extrapolator_->AddOdometryData(odometry_data);
}

std::unique_ptr<LocalTrajectoryBuilder3D::InsertionResult>
LocalTrajectoryBuilder3D::InsertIntoSubmap(
    const common::Time time,
    const sensor::RangeData& filtered_range_data_in_local,
    const sensor::RangeData& filtered_range_data_in_tracking,
    const sensor::PointCloud& high_resolution_point_cloud_in_tracking,
    const sensor::PointCloud& low_resolution_point_cloud_in_tracking,
    const transform::Rigid3d& pose_estimate,
    const Eigen::Quaterniond& gravity_alignment) {
  if (motion_filter_.IsSimilar(time, pose_estimate)) {
    return nullptr;
  }
  // Querying the active submaps must be done here before calling
  // InsertRangeData() since the queried values are valid for next insertion.
  std::vector<std::shared_ptr<const mapping::Submap3D>> insertion_submaps;
  for (const std::shared_ptr<mapping::Submap3D>& submap :
       active_submaps_.submaps()) {
    insertion_submaps.push_back(submap);
  }
  active_submaps_.InsertRangeData(filtered_range_data_in_local,
                                  gravity_alignment);

  // (cxn) Pbaselink 为去畸变后 点云(参考系为baselink)，R为 baselink 在 IMU系下 旋转
  // 对 R×Pbaselink得到的点云 统计直方图：[0,pi]分成了120个bin，每个bin上都有float权值
  const Eigen::VectorXf rotational_scan_matcher_histogram =
      scan_matching::RotationalScanMatcher::ComputeHistogram(
          sensor::TransformPointCloud(
              filtered_range_data_in_tracking.returns,
              transform::Rigid3f::Rotation(gravity_alignment.cast<float>())),
          options_.rotational_histogram_size());

  return common::make_unique<InsertionResult>(
      InsertionResult{std::make_shared<const mapping::TrajectoryNode::Data>(
                          mapping::TrajectoryNode::Data{
                              time,
                              gravity_alignment,
                              {},  // 'filtered_point_cloud' is only used in 2D.
                              high_resolution_point_cloud_in_tracking,
                              low_resolution_point_cloud_in_tracking,
                              rotational_scan_matcher_histogram,
                              pose_estimate}),
                      std::move(insertion_submaps)});
}

void LocalTrajectoryBuilder3D::RegisterMetrics(
    metrics::FamilyFactory* family_factory) {
  auto* latency = family_factory->NewGaugeFamily(
      "mapping_internal_3d_local_trajectory_builder_latency",
      "Duration from first incoming point cloud in accumulation to local slam "
      "result");
  kLocalSlamLatencyMetric = latency->Add({});
  auto score_boundaries = metrics::Histogram::FixedWidth(0.05, 20);
  auto* scores = family_factory->NewHistogramFamily(
      "mapping_internal_3d_local_trajectory_builder_scores",
      "Local scan matcher scores", score_boundaries);
  kRealTimeCorrelativeScanMatcherScoreMetric =
      scores->Add({{"scan_matcher", "real_time_correlative"}});
  auto cost_boundaries = metrics::Histogram::ScaledPowersOf(2, 0.01, 100);
  auto* costs = family_factory->NewHistogramFamily(
      "mapping_internal_3d_local_trajectory_builder_costs",
      "Local scan matcher costs", cost_boundaries);
  kCeresScanMatcherCostMetric = costs->Add({{"scan_matcher", "ceres"}});
  auto distance_boundaries = metrics::Histogram::ScaledPowersOf(2, 0.01, 10);
  auto* residuals = family_factory->NewHistogramFamily(
      "mapping_internal_3d_local_trajectory_builder_residuals",
      "Local scan matcher residuals", distance_boundaries);
  kScanMatcherResidualDistanceMetric =
      residuals->Add({{"component", "distance"}});
  kScanMatcherResidualAngleMetric = residuals->Add({{"component", "angle"}});
}

}  // namespace mapping
}  // namespace cartographer

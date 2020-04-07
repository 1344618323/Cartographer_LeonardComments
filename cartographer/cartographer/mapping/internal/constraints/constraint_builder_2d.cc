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

#include "cartographer/mapping/internal/constraints/constraint_builder_2d.h"

#include <cmath>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
#include <string>

#include "Eigen/Eigenvalues"
#include "cartographer/common/make_unique.h"
#include "cartographer/common/math.h"
#include "cartographer/common/thread_pool.h"
#include "cartographer/mapping/proto/scan_matching//ceres_scan_matcher_options_2d.pb.h"
#include "cartographer/mapping/proto/scan_matching//fast_correlative_scan_matcher_options_2d.pb.h"
#include "cartographer/metrics/counter.h"
#include "cartographer/metrics/gauge.h"
#include "cartographer/metrics/histogram.h"
#include "cartographer/transform/transform.h"
#include "glog/logging.h"

/*(cxn)pose-graph.lua中的默认配置

constraint_builder = {
  sampling_ratio = 0.3,
  max_constraint_distance = 15.,
  min_score = 0.55,
  global_localization_min_score = 0.6,
  loop_closure_translation_weight = 1.1e4,
  loop_closure_rotation_weight = 1e5,
  log_matches = true,
  
  fast_correlative_scan_matcher = {
    linear_search_window = 7.,
    angular_search_window = math.rad(30.),
    branch_and_bound_depth = 7,
  },
  
  ceres_scan_matcher = {
    occupied_space_weight = 20.,
    translation_weight = 10.,
    rotation_weight = 1.,
    ceres_solver_options = {
        use_nonmonotonic_steps = true,
        max_num_iterations = 10,
        num_threads = 1,
    },
  },
  
  fast_correlative_scan_matcher_3d = {
    branch_and_bound_depth = 8,
    full_resolution_depth = 3,
    min_rotational_score = 0.77,
    min_low_resolution_score = 0.55,
    linear_xy_search_window = 5.,
    linear_z_search_window = 1.,
    angular_search_window = math.rad(15.),
  },

  ceres_scan_matcher_3d = {
    occupied_space_weight_0 = 5.,
    occupied_space_weight_1 = 30.,
    translation_weight = 10.,
    rotation_weight = 1.,
    only_optimize_yaw = false,
    ceres_solver_options = {
        use_nonmonotonic_steps = false,
        max_num_iterations = 10,
        num_threads = 1,
    },
  },
},
*/

namespace cartographer {
namespace mapping {
namespace constraints {

static auto* kConstraintsSearchedMetric = metrics::Counter::Null();
static auto* kConstraintsFoundMetric = metrics::Counter::Null();
static auto* kGlobalConstraintsSearchedMetric = metrics::Counter::Null();
static auto* kGlobalConstraintsFoundMetric = metrics::Counter::Null();
static auto* kQueueLengthMetric = metrics::Gauge::Null();
static auto* kConstraintScoresMetric = metrics::Histogram::Null();
static auto* kGlobalConstraintScoresMetric = metrics::Histogram::Null();

transform::Rigid2d ComputeSubmapPose(const Submap2D& submap) {
  return transform::Project2D(submap.local_pose());
}

ConstraintBuilder2D::ConstraintBuilder2D(
    const constraints::proto::ConstraintBuilderOptions& options,
    common::ThreadPoolInterface* const thread_pool)
    : options_(options),
      thread_pool_(thread_pool),
      finish_node_task_(common::make_unique<common::Task>()),
      when_done_task_(common::make_unique<common::Task>()),
      sampler_(options.sampling_ratio()),//2d.lua中为0.3
      ceres_scan_matcher_(options.ceres_scan_matcher_options()) {}

ConstraintBuilder2D::~ConstraintBuilder2D() {
  common::MutexLocker locker(&mutex_);
  CHECK_EQ(finish_node_task_->GetState(), common::Task::NEW);
  CHECK_EQ(when_done_task_->GetState(), common::Task::NEW);
  CHECK_EQ(constraints_.size(), 0) << "WhenDone() was not called";
  CHECK_EQ(num_started_nodes_, num_finished_nodes_);
  CHECK(when_done_ == nullptr);
}

//node与submap同属一个轨迹 or 两个轨迹前不久刚刚绑定过一次
void ConstraintBuilder2D::MaybeAddConstraint(
    const SubmapId& submap_id, const Submap2D* const submap,
    const NodeId& node_id, const TrajectoryNode::Data* const constant_data,
    const transform::Rigid2d& initial_relative_pose) {

  //从初值判断就发现submap与node位移太大（2d.lua中为15m）就不认为有约束
  if (initial_relative_pose.translation().norm() >
      options_.max_constraint_distance()) {
    return;
  }

  //抽样，少处理一些数据
  if (!sampler_.Pulse()) return;

  common::MutexLocker locker(&mutex_);
  if (when_done_) {
    LOG(WARNING)
        << "MaybeAddConstraint was called while WhenDone was scheduled.";
  }
  constraints_.emplace_back();
  kQueueLengthMetric->Set(constraints_.size());
  auto* const constraint = &constraints_.back();

  //新建分枝定界匹配器
  const auto* scan_matcher =
      DispatchScanMatcherConstruction(submap_id, submap->grid());
  
  auto constraint_task = common::make_unique<common::Task>();
  constraint_task->SetWorkItem([=]() EXCLUDES(mutex_) {
    ComputeConstraint(submap_id, submap, node_id, false, /* match_full_submap */
                      constant_data, initial_relative_pose, *scan_matcher,
                      constraint);
  });
  constraint_task->AddDependency(scan_matcher->creation_task_handle);
  auto constraint_task_handle =
      thread_pool_->Schedule(std::move(constraint_task));
  finish_node_task_->AddDependency(constraint_task_handle);
}

//用于submap与node属不同轨迹的情况，且前一段时间也没有绑定过
void ConstraintBuilder2D::MaybeAddGlobalConstraint(
    const SubmapId& submap_id, const Submap2D* const submap,
    const NodeId& node_id, const TrajectoryNode::Data* const constant_data) {
  common::MutexLocker locker(&mutex_);
  if (when_done_) {
    LOG(WARNING)
        << "MaybeAddGlobalConstraint was called while WhenDone was scheduled.";
  }
  constraints_.emplace_back();
  kQueueLengthMetric->Set(constraints_.size());
  auto* const constraint = &constraints_.back();
  const auto* scan_matcher =
      DispatchScanMatcherConstruction(submap_id, submap->grid());
  auto constraint_task = common::make_unique<common::Task>();
  constraint_task->SetWorkItem([=]() EXCLUDES(mutex_) {
    ComputeConstraint(submap_id, submap, node_id, true, /* match_full_submap */
                      constant_data, transform::Rigid2d::Identity(),
                      *scan_matcher, constraint);
  });
  constraint_task->AddDependency(scan_matcher->creation_task_handle);
  auto constraint_task_handle =
      thread_pool_->Schedule(std::move(constraint_task));
  finish_node_task_->AddDependency(constraint_task_handle);
}

void ConstraintBuilder2D::NotifyEndOfNode() {
  common::MutexLocker locker(&mutex_);
  CHECK(finish_node_task_ != nullptr);
  finish_node_task_->SetWorkItem([this] {
    common::MutexLocker locker(&mutex_);
    ++num_finished_nodes_;
  });
  auto finish_node_task_handle =
      thread_pool_->Schedule(std::move(finish_node_task_));
  finish_node_task_ = common::make_unique<common::Task>();
  when_done_task_->AddDependency(finish_node_task_handle);
  ++num_started_nodes_;
}

void ConstraintBuilder2D::WhenDone(
    const std::function<void(const ConstraintBuilder2D::Result&)>& callback) {
  common::MutexLocker locker(&mutex_);
  CHECK(when_done_ == nullptr);
  // TODO(gaschler): Consider using just std::function, it can also be empty.
  when_done_ =
      common::make_unique<std::function<void(const Result&)>>(callback);
  CHECK(when_done_task_ != nullptr);
  when_done_task_->SetWorkItem([this] { RunWhenDoneCallback(); });
  thread_pool_->Schedule(std::move(when_done_task_));
  when_done_task_ = common::make_unique<common::Task>();
}

const ConstraintBuilder2D::SubmapScanMatcher*
ConstraintBuilder2D::DispatchScanMatcherConstruction(const SubmapId& submap_id,
                                                     const Grid2D* const grid) {
  if (submap_scan_matchers_.count(submap_id) != 0) {
    return &submap_scan_matchers_.at(submap_id);
  }
  auto& submap_scan_matcher = submap_scan_matchers_[submap_id];
  submap_scan_matcher.grid = grid;
  auto& scan_matcher_options = options_.fast_correlative_scan_matcher_options();
  auto scan_matcher_task = common::make_unique<common::Task>();
  scan_matcher_task->SetWorkItem(
      [&submap_scan_matcher, &scan_matcher_options]() {
        submap_scan_matcher.fast_correlative_scan_matcher =
            common::make_unique<scan_matching::FastCorrelativeScanMatcher2D>(
                *submap_scan_matcher.grid, scan_matcher_options);
      });
  submap_scan_matcher.creation_task_handle =
      thread_pool_->Schedule(std::move(scan_matcher_task));
  return &submap_scan_matchers_.at(submap_id);
}

void ConstraintBuilder2D::ComputeConstraint(
    const SubmapId& submap_id, const Submap2D* const submap,
    const NodeId& node_id, bool match_full_submap,
    const TrajectoryNode::Data* const constant_data,
    const transform::Rigid2d& initial_relative_pose,
    const SubmapScanMatcher& submap_scan_matcher,
    std::unique_ptr<ConstraintBuilder2D::Constraint>* constraint) {

  // node位姿的初值为 Tlw-node-2d = Tlw_submap * Tgw-submap'*Tgw-node-2d
  // 我们的目标是算出 node在 该 submap的local-frame下 的位姿 （node-2d表示少了一个IMU的yaw角）
  const transform::Rigid2d initial_pose =
      ComputeSubmapPose(*submap) * initial_relative_pose;

  // The 'constraint_transform' (submap i <- node j) is computed from:
  // - a 'filtered_gravity_aligned_point_cloud' in node j,
  // - the initial guess 'initial_pose' for (map <- node j),
  // - the result 'pose_estimate' of Match() (map <- node j).
  // - the ComputeSubmapPose() (map <- submap i)
  float score = 0.;
  transform::Rigid2d pose_estimate = transform::Rigid2d::Identity();

  // Compute 'pose_estimate' in three stages:
  // 1. Fast estimate using the fast correlative scan matcher.
  // 2. Prune if the score is too low.
  // 3. Refine.
  if (match_full_submap) {
    kGlobalConstraintsSearchedMetric->Increment();
    if (submap_scan_matcher.fast_correlative_scan_matcher->MatchFullSubmap(
            constant_data->filtered_gravity_aligned_point_cloud,
            options_.global_localization_min_score(), &score, &pose_estimate)) {
      CHECK_GT(score, options_.global_localization_min_score());
      CHECK_GE(node_id.trajectory_id, 0);
      CHECK_GE(submap_id.trajectory_id, 0);
      kGlobalConstraintsFoundMetric->Increment();
      kGlobalConstraintScoresMetric->Observe(score);
    } else {
      return;
    }
  } else {
    kConstraintsSearchedMetric->Increment();
    //分枝定界法求解
    if (submap_scan_matcher.fast_correlative_scan_matcher->Match(
            initial_pose, constant_data->filtered_gravity_aligned_point_cloud,
            options_.min_score(), &score, &pose_estimate)) {
      // We've reported a successful local match.
      CHECK_GT(score, options_.min_score());
      kConstraintsFoundMetric->Increment();
      kConstraintScoresMetric->Observe(score);
    } else {
      //解不出来，说明匹配不上
      return;
    }
  }
  {
    common::MutexLocker locker(&mutex_);
    score_histogram_.Add(score);
  }

  // Use the CSM estimate as both the initial and previous pose. This has the
  // effect that, in the absence of better information, we prefer the original
  // CSM estimate.
  ceres::Solver::Summary unused_summary;
  ceres_scan_matcher_.Match(pose_estimate.translation(), pose_estimate,
                            constant_data->filtered_gravity_aligned_point_cloud,
                            *submap_scan_matcher.grid, &pose_estimate,
                            &unused_summary);

  //算出submap与node-2d的约束
  const transform::Rigid2d constraint_transform =
      ComputeSubmapPose(*submap).inverse() * pose_estimate;
  constraint->reset(new Constraint{submap_id,
                                   node_id,
                                   {transform::Embed3D(constraint_transform),
                                    options_.loop_closure_translation_weight(),
                                    options_.loop_closure_rotation_weight()},
                                   Constraint::INTER_SUBMAP});

  if (options_.log_matches()) {
    std::ostringstream info;
    info << "Node " << node_id << " with "
         << constant_data->filtered_gravity_aligned_point_cloud.size()
         << " points on submap " << submap_id << std::fixed;
    if (match_full_submap) {
      info << " matches";
    } else {
      const transform::Rigid2d difference =
          initial_pose.inverse() * pose_estimate;
      info << " differs by translation " << std::setprecision(2)
           << difference.translation().norm() << " rotation "
           << std::setprecision(3) << std::abs(difference.normalized_angle());
    }
    info << " with score " << std::setprecision(1) << 100. * score << "%.";
    LOG(INFO) << info.str();
  }
}

void ConstraintBuilder2D::RunWhenDoneCallback() {
  Result result;
  std::unique_ptr<std::function<void(const Result&)>> callback;
  {
    common::MutexLocker locker(&mutex_);
    CHECK(when_done_ != nullptr);
    for (const std::unique_ptr<Constraint>& constraint : constraints_) {
      if (constraint == nullptr) continue;
      result.push_back(*constraint);
    }
    if (options_.log_matches()) {
      LOG(INFO) << constraints_.size() << " computations resulted in "
                << result.size() << " additional constraints.";
      LOG(INFO) << "Score histogram:\n" << score_histogram_.ToString(10);
    }
    constraints_.clear();
    callback = std::move(when_done_);
    when_done_.reset();
    kQueueLengthMetric->Set(constraints_.size());
  }
  (*callback)(result);
}

int ConstraintBuilder2D::GetNumFinishedNodes() {
  common::MutexLocker locker(&mutex_);
  return num_finished_nodes_;
}

void ConstraintBuilder2D::DeleteScanMatcher(const SubmapId& submap_id) {
  common::MutexLocker locker(&mutex_);
  if (when_done_) {
    LOG(WARNING)
        << "DeleteScanMatcher was called while WhenDone was scheduled.";
  }
  submap_scan_matchers_.erase(submap_id);
}

void ConstraintBuilder2D::RegisterMetrics(metrics::FamilyFactory* factory) {
  auto* counts = factory->NewCounterFamily(
      "mapping_internal_constraints_constraint_builder_2d_constraints",
      "Constraints computed");
  kConstraintsSearchedMetric =
      counts->Add({{"search_region", "local"}, {"matcher", "searched"}});
  kConstraintsFoundMetric =
      counts->Add({{"search_region", "local"}, {"matcher", "found"}});
  kGlobalConstraintsSearchedMetric =
      counts->Add({{"search_region", "global"}, {"matcher", "searched"}});
  kGlobalConstraintsFoundMetric =
      counts->Add({{"search_region", "global"}, {"matcher", "found"}});
  auto* queue_length = factory->NewGaugeFamily(
      "mapping_internal_constraints_constraint_builder_2d_queue_length",
      "Queue length");
  kQueueLengthMetric = queue_length->Add({});
  auto boundaries = metrics::Histogram::FixedWidth(0.05, 20);
  auto* scores = factory->NewHistogramFamily(
      "mapping_internal_constraints_constraint_builder_2d_scores",
      "Constraint scores built", boundaries);
  kConstraintScoresMetric = scores->Add({{"search_region", "local"}});
  kGlobalConstraintScoresMetric = scores->Add({{"search_region", "global"}});
}

}  // namespace constraints
}  // namespace mapping
}  // namespace cartographer

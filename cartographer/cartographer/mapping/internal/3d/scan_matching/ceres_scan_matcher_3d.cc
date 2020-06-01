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

#include "cartographer/mapping/internal/3d/scan_matching/ceres_scan_matcher_3d.h"

#include <string>
#include <utility>
#include <vector>

#include "cartographer/common/ceres_solver_options.h"
#include "cartographer/common/make_unique.h"
#include "cartographer/mapping/internal/3d/rotation_parameterization.h"
#include "cartographer/mapping/internal/3d/scan_matching/occupied_space_cost_function_3d.h"
#include "cartographer/mapping/internal/3d/scan_matching/rotation_delta_cost_functor_3d.h"
#include "cartographer/mapping/internal/3d/scan_matching/translation_delta_cost_functor_3d.h"
#include "cartographer/mapping/internal/optimization/ceres_pose.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/transform.h"
#include "ceres/ceres.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {
namespace scan_matching {

proto::CeresScanMatcherOptions3D CreateCeresScanMatcherOptions3D(
    common::LuaParameterDictionary* const parameter_dictionary) {
  proto::CeresScanMatcherOptions3D options;
  for (int i = 0;; ++i) {
    const std::string lua_identifier =
        "occupied_space_weight_" + std::to_string(i);
    if (!parameter_dictionary->HasKey(lua_identifier)) {
      break;
    }
    options.add_occupied_space_weight(
        parameter_dictionary->GetDouble(lua_identifier));
  }
  options.set_translation_weight(
      parameter_dictionary->GetDouble("translation_weight"));
  options.set_rotation_weight(
      parameter_dictionary->GetDouble("rotation_weight"));
  options.set_only_optimize_yaw(
      parameter_dictionary->GetBool("only_optimize_yaw"));
  *options.mutable_ceres_solver_options() =
      common::CreateCeresSolverOptionsProto(
          parameter_dictionary->GetDictionary("ceres_solver_options").get());
  return options;
}

CeresScanMatcher3D::CeresScanMatcher3D(
    const proto::CeresScanMatcherOptions3D& options)
    : options_(options),
      ceres_solver_options_(
          common::CreateCeresSolverOptions(options.ceres_solver_options())) {
  ceres_solver_options_.linear_solver_type = ceres::DENSE_QR;
}

void CeresScanMatcher3D::Match(
    const Eigen::Vector3d& target_translation,
    const transform::Rigid3d& initial_pose_estimate,
    const std::vector<PointCloudAndHybridGridPointers>&
        point_clouds_and_hybrid_grids,
    transform::Rigid3d* const pose_estimate,
    ceres::Solver::Summary* const summary) {
  ceres::Problem problem;
  //(cxn) LocalParameterization可以参考链接：http://www.ceres-solver.org/nnls_modeling.html?highlight=localparameterization#_CPPv2N5ceres21LocalParameterizationE
  //有时，待优化参数x可能会过度参数化问题。在这种情况下，最好选择一个参数化来移除成本的空方向
  //比如说四元数用来描述一个旋转向量（3维），却用了四个参数，那么对于 x+\deltax 这个4*1向量而言，有一个分量的增长就是无意义的
  //考虑到四元数的常用性，ceres库实现了 ceres::QuaternionParameterization 继承了 LocalParameterization
  //其中的优化向量x是3维的，而计算的增量delta是3维的，这个类中实现了：计算优化量x的雅克比矩阵、优化量x的plus()函数
  optimization::CeresPose ceres_pose(
      initial_pose_estimate, nullptr /* translation_parameterization */,
      options_.only_optimize_yaw()
          ? std::unique_ptr<ceres::LocalParameterization>(
                common::make_unique<ceres::AutoDiffLocalParameterization<
                    YawOnlyQuaternionPlus, 4, 1>>())
          : std::unique_ptr<ceres::LocalParameterization>(
                common::make_unique<ceres::QuaternionParameterization>()),
      &problem);

  CHECK_EQ(options_.occupied_space_weight_size(),
           point_clouds_and_hybrid_grids.size());
  for (size_t i = 0; i != point_clouds_and_hybrid_grids.size(); ++i) {
    CHECK_GT(options_.occupied_space_weight(i), 0.);
    const sensor::PointCloud& point_cloud =
        *point_clouds_and_hybrid_grids[i].first;
    const HybridGrid& hybrid_grid = *point_clouds_and_hybrid_grids[i].second;
    //(cxn)？ 默认配置中 occupied-space-weight-0 = 1.（高分辨率）, occupied-space-weight-1 = 6.（低分辨率）
    // 与直觉不符，为啥呢？官网给了个说明：https://google-cartographer-ros.readthedocs.io/en/latest/algo_walkthrough.html?highlight=high_resolution#local-slam
    // Scan matching starts by aligning far points of the low resolution point cloud with the low resolution hybrid grid 
    // and then refines the pose by aligning the close high resolution points with the high resolution hybrid grid.
    problem.AddResidualBlock(
        OccupiedSpaceCostFunction3D::CreateAutoDiffCostFunction(
            options_.occupied_space_weight(i) /
                std::sqrt(static_cast<double>(point_cloud.size())),
            point_cloud, hybrid_grid),
        nullptr /* loss function */, ceres_pose.translation(),
        ceres_pose.rotation());
  }
  CHECK_GT(options_.translation_weight(), 0.);
  problem.AddResidualBlock(
      TranslationDeltaCostFunctor3D::CreateAutoDiffCostFunction(
          options_.translation_weight(), target_translation),
      nullptr /* loss function */, ceres_pose.translation());
  CHECK_GT(options_.rotation_weight(), 0.);
  problem.AddResidualBlock(
      RotationDeltaCostFunctor3D::CreateAutoDiffCostFunction(
          options_.rotation_weight(), initial_pose_estimate.rotation()),
      nullptr /* loss function */, ceres_pose.rotation());

  ceres::Solve(ceres_solver_options_, &problem, summary);

  *pose_estimate = ceres_pose.ToRigid();
}

}  // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer

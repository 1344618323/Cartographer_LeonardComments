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

#include "cartographer/mapping/internal/2d/scan_matching/real_time_correlative_scan_matcher_2d.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>

#include "Eigen/Geometry"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/math.h"
#include "cartographer/mapping/2d/probability_grid.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/transform/transform.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {
namespace scan_matching {


/*(cxn)
2d.lua配置

linear_search_window = 0.1,
angular_search_window = math.rad(20.),
translation_delta_cost_weight = 1e-1,
rotation_delta_cost_weight = 1e-1,

意味着 要在 -20°～20°，-0.1~0.1，-0.1~0.1 内搜索位姿


总结一下cms的实现

输入一帧点云S，预测初值为 Tinit，地图M，地图分辨率r

根据 S 中的最长测量，决定 角度的步长 a-step，在-20°～20°中搜索，假设有n组；x-step，y-step=0.1/r

将点云S用 Tinit 变换，并分别旋转变换算出 -20°～20°之后的点云，得到 {S_r0,S_r1,..., S_rn}，算出栅格坐标（整数坐标）

对x有搜索范围 -0.1/r ~ 0.1/r（整数），如果r=0.05m，x的搜索范围为 vx=[-2,-1，0,1，2]
对y同理

遍历{S_r0,S_r1,..., S_rn}： i=0～n
    for mx:vx
        for my:vy
            遍历S_r0中每一束激光 j=0～k
                设S_ri中第j个激光栅格坐标为 S_ri_j
                        sj = score[S_ri_j+(mx,my)]   ：这个score就是返回该栅格左边的概率值
            score = sum(sj)/k
            score*=exp(-d^2) 即位姿与初值Tinit差越大，分数降的越多

找能使score最大的那个 位姿 {ri,mx,my}

最终输出为  Tinit.translate+(mx,my)  Tinit.R*ri（对于2d而言，旋转矩阵的左乘右乘没区别）
按照我的理解 应该是 ri* Tinit.R，不过对2d而言无所谓，有空看3d的时候，再琢磨下
*/


RealTimeCorrelativeScanMatcher2D::RealTimeCorrelativeScanMatcher2D(
    const proto::RealTimeCorrelativeScanMatcherOptions& options)
    : options_(options) {}


/**
 * 计算出来所有的要进行搜索的解。
 * 即得到三层for循环的所有的组合
 */
std::vector<Candidate2D>
RealTimeCorrelativeScanMatcher2D::GenerateExhaustiveSearchCandidates(
    const SearchParameters& search_parameters) const {
  int num_candidates = 0;
  for (int scan_index = 0; scan_index != search_parameters.num_scans;
       ++scan_index) {
    const int num_linear_x_candidates =
        (search_parameters.linear_bounds[scan_index].max_x -
         search_parameters.linear_bounds[scan_index].min_x + 1);
    const int num_linear_y_candidates =
        (search_parameters.linear_bounds[scan_index].max_y -
         search_parameters.linear_bounds[scan_index].min_y + 1);
    num_candidates += num_linear_x_candidates * num_linear_y_candidates;
  }
  std::vector<Candidate2D> candidates;
  candidates.reserve(num_candidates);
  for (int scan_index = 0; scan_index != search_parameters.num_scans;
       ++scan_index) {
    for (int x_index_offset = search_parameters.linear_bounds[scan_index].min_x;
         x_index_offset <= search_parameters.linear_bounds[scan_index].max_x;
         ++x_index_offset) {
      for (int y_index_offset =
               search_parameters.linear_bounds[scan_index].min_y;
           y_index_offset <= search_parameters.linear_bounds[scan_index].max_y;
           ++y_index_offset) {
        candidates.emplace_back(scan_index, x_index_offset, y_index_offset,
                                search_parameters);
      }
    }
  }
  CHECK_EQ(candidates.size(), num_candidates);
  return candidates;
}

double RealTimeCorrelativeScanMatcher2D::Match(
    const transform::Rigid2d& initial_pose_estimate,
    const sensor::PointCloud& point_cloud,
    const ProbabilityGrid& probability_grid,
    transform::Rigid2d* pose_estimate) const {
  CHECK_NOTNULL(pose_estimate);

  //(cxn)把点云旋转至初值
  const Eigen::Rotation2Dd initial_rotation = initial_pose_estimate.rotation();
  const sensor::PointCloud rotated_point_cloud = sensor::TransformPointCloud(
      point_cloud,
      transform::Rigid3f::Rotation(Eigen::AngleAxisf(
          initial_rotation.cast<float>().angle(), Eigen::Vector3f::UnitZ())));
  
  const SearchParameters search_parameters(
      options_.linear_search_window(), options_.angular_search_window(),
      rotated_point_cloud, probability_grid.limits().resolution());

  //依次旋转点云
  const std::vector<sensor::PointCloud> rotated_scans =
      GenerateRotatedScans(rotated_point_cloud, search_parameters);

  //平移点云至初值
  const std::vector<DiscreteScan2D> discrete_scans = DiscretizeScans(
      probability_grid.limits(), rotated_scans,
      Eigen::Translation2f(initial_pose_estimate.translation().x(),
                           initial_pose_estimate.translation().y()));
  
  std::vector<Candidate2D> candidates =
      GenerateExhaustiveSearchCandidates(search_parameters);

  ScoreCandidates(probability_grid, discrete_scans, search_parameters,
                  &candidates);

  const Candidate2D& best_candidate =
      *std::max_element(candidates.begin(), candidates.end());
  *pose_estimate = transform::Rigid2d(
      {initial_pose_estimate.translation().x() + best_candidate.x,
       initial_pose_estimate.translation().y() + best_candidate.y},
      initial_rotation * Eigen::Rotation2Dd(best_candidate.orientation));
  return best_candidate.score;
}

void RealTimeCorrelativeScanMatcher2D::ScoreCandidates(
    const ProbabilityGrid& probability_grid,
    const std::vector<DiscreteScan2D>& discrete_scans,
    const SearchParameters& search_parameters,
    std::vector<Candidate2D>* const candidates) const {

  //枚举所有的candidate
  //每个candidate都暴包括一个 angle-offset、x-offset、y-offset
  for (Candidate2D& candidate : *candidates) {
    candidate.score = 0.f;

    //每个candidate都需要枚举所有的点云
    //另外点云在 这个 angle-offset 下的坐标都是 提前算好的
    for (const Eigen::Array2i& xy_index :
         discrete_scans[candidate.scan_index]) {
    
      //所有点云都是已旋转过了，所以这里只需要相加就可以
      const Eigen::Array2i proposed_xy_index(
          xy_index.x() + candidate.x_index_offset,
          xy_index.y() + candidate.y_index_offset);
      const float probability =
          probability_grid.GetProbability(proposed_xy_index);
      candidate.score += probability;
    }

    //进行平均
    candidate.score /=
        static_cast<float>(discrete_scans[candidate.scan_index].size());

    //施加惩罚 离初始位置越远 惩罚的越严重
    candidate.score *=
        std::exp(-common::Pow2(std::hypot(candidate.x, candidate.y) *
                                   options_.translation_delta_cost_weight() +
                               std::abs(candidate.orientation) *
                                   options_.rotation_delta_cost_weight()));
    CHECK_GT(candidate.score, 0.f);
  }
}

}  // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer

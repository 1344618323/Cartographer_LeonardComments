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

#include "cartographer/sensor/internal/voxel_filter.h"

#include <cmath>

#include "cartographer/common/math.h"

namespace cartographer {
namespace sensor {

namespace {

PointCloud FilterByMaxRange(const PointCloud& point_cloud,
                            const float max_range) {
  PointCloud result;
  for (const Eigen::Vector3f& point : point_cloud) {
    if (point.norm() <= max_range) {
      result.push_back(point);
    }
  }
  return result;
}

PointCloud AdaptivelyVoxelFiltered(
    const proto::AdaptiveVoxelFilterOptions& options,
    const PointCloud& point_cloud) {
      //(cxn)1.如果点云本来就少，放回源数据就少了
      //2.用尺寸比较大的体素滤波器（0.5m），若点还足够多，返回滤波结果
      //3.不断对highlength从maxlenghth开始循环二分，直到分到 highlength=0.01*maxlength，lowlength=highlength/2
      // 若找到了能得到足够多点云的lowlength，就用二分法在 [lowlength,highlength] 中找最佳尺寸
  if (point_cloud.size() <= options.min_num_points()) {
    // 'point_cloud' is already sparse enough.
    return point_cloud;
  }
  PointCloud result = VoxelFilter(options.max_length()).Filter(point_cloud);
  if (result.size() >= options.min_num_points()) {
    // Filtering with 'max_length' resulted in a sufficiently dense point cloud.
    return result;
  }
  // Search for a 'low_length' that is known to result in a sufficiently
  // dense point cloud. We give up and use the full 'point_cloud' if reducing
  // the edge length by a factor of 1e-2 is not enough.
  for (float high_length = options.max_length();
       high_length > 1e-2f * options.max_length(); high_length /= 2.f) {
    float low_length = high_length / 2.f;
    result = VoxelFilter(low_length).Filter(point_cloud);
    if (result.size() >= options.min_num_points()) {
      // Binary search to find the right amount of filtering. 'low_length' gave
      // a sufficiently dense 'result', 'high_length' did not. We stop when the
      // edge length is at most 10% off.
      //(cxn)二分法找最佳体素滤波尺寸，知道high是low的1.1倍为止
      while ((high_length - low_length) / low_length > 1e-1f) {
        const float mid_length = (low_length + high_length) / 2.f;
        const PointCloud candidate =
            VoxelFilter(mid_length).Filter(point_cloud);
        if (candidate.size() >= options.min_num_points()) {
          low_length = mid_length;
          result = candidate;
        } else {
          high_length = mid_length;
        }
      }
      return result;
    }
  }
  return result;
}

}  // namespace


//(cxn)可以看出体素滤波是通过 无序集unordered_set 实现的
PointCloud VoxelFilter::Filter(const PointCloud& point_cloud) {
  PointCloud results;
  for (const Eigen::Vector3f& point : point_cloud) {
    auto it_inserted = voxel_set_.insert(IndexToKey(GetCellIndex(point)));
    if (it_inserted.second) {
      results.push_back(point);
    }
  }
  return results;
}

TimedPointCloud VoxelFilter::Filter(const TimedPointCloud& timed_point_cloud) {
  TimedPointCloud results;
  for (const Eigen::Vector4f& point : timed_point_cloud) {
    auto it_inserted =
        voxel_set_.insert(IndexToKey(GetCellIndex(point.head<3>())));
    if (it_inserted.second) {
      results.push_back(point);
    }
  }
  return results;
}

std::vector<sensor::TimedPointCloudOriginData::RangeMeasurement>
VoxelFilter::Filter(
    const std::vector<sensor::TimedPointCloudOriginData::RangeMeasurement>&
        range_measurements) {
  std::vector<sensor::TimedPointCloudOriginData::RangeMeasurement> results;
  for (const auto& range_measurement : range_measurements) {
    auto it_inserted = voxel_set_.insert(
        IndexToKey(GetCellIndex(range_measurement.point_time.head<3>())));
    if (it_inserted.second) {
      results.push_back(range_measurement);
    }
  }
  return results;
}

VoxelFilter::KeyType VoxelFilter::IndexToKey(const Eigen::Array3i& index) {
  KeyType k_0(static_cast<uint32>(index[0]));
  KeyType k_1(static_cast<uint32>(index[1]));
  KeyType k_2(static_cast<uint32>(index[2]));
  return (k_0 << 2 * 32) | (k_1 << 1 * 32) | k_2;
}

Eigen::Array3i VoxelFilter::GetCellIndex(const Eigen::Vector3f& point) const {
  Eigen::Array3f index = point.array() / resolution_;
  return Eigen::Array3i(common::RoundToInt(index.x()),
                        common::RoundToInt(index.y()),
                        common::RoundToInt(index.z()));
}

proto::AdaptiveVoxelFilterOptions CreateAdaptiveVoxelFilterOptions(
    common::LuaParameterDictionary* const parameter_dictionary) {
  proto::AdaptiveVoxelFilterOptions options;
  options.set_max_length(parameter_dictionary->GetDouble("max_length"));
  options.set_min_num_points(
      parameter_dictionary->GetNonNegativeInt("min_num_points"));
  options.set_max_range(parameter_dictionary->GetDouble("max_range"));
  return options;
}

AdaptiveVoxelFilter::AdaptiveVoxelFilter(
    const proto::AdaptiveVoxelFilterOptions& options)
    : options_(options) {}

PointCloud AdaptiveVoxelFilter::Filter(const PointCloud& point_cloud) const {
  return AdaptivelyVoxelFiltered(
      options_, FilterByMaxRange(point_cloud, options_.max_range()));
}

}  // namespace sensor
}  // namespace cartographer

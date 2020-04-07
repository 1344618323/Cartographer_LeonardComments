/*
 * Copyright 2018 The Cartographer Authors
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

#include "cartographer/mapping/internal/range_data_collator.h"

#include <memory>

#include "cartographer/common/make_unique.h"
#include "cartographer/mapping/local_slam_result_data.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {

//(cxn)这个文件的作用是多个雷达数据同步
/* 如两个雷达 A、B

A:  t0a|------------------|t1a  t2a|---------------------|t3a t4a|---------------------|t5a
B:                 t0b|------------------|t1b  t2b|---------------------|t3b

第一次调用Add_RangeData时，加入的是截止到t1a的A雷达数据，不做反应

第二次调用Add_RangeData时，加入的是截止到t1b的B雷达数据，进入处理函数CropAndMerge
打包 t0a～t1a的所有雷达数据作为结果

第三次调用Add_RangeData时，加入的是截止到t3a的A雷达数据，进入处理函数CropAndMerge
打包 t1a～t1b的所有雷达数据作为结果

第四次调用Add_RangeData时，加入的是截止到t3b的B雷达数据，进入处理函数CropAndMerge
打包 t1b～t3a的所有雷达数据作为结果

有一个map 用来作为存储 多个雷达数据 的缓存，map[A]、map[B]:这些键都只能存一段数据
如 map[A]，只能存 t0a～t1a的数据
此时，有两中情况发生
1. 如 t0b～t1b的数据来了，这是map中所有键都值了，
    就看第一个有值的键的最后一个时间戳，即t1a之前的值都清了（对map[B]相当于清了t0b～t1a的数据）
2. 或者说  t0b～t1b的数据还没来，t2a～t3a的数据来了，我们也必须把 t1a之前的数据清理掉了
3. 最合理的情况应该是，t0b～t1b来了，清了t1a之前的数据，然后t2a～t3a的数据再来，
    此时第一个有值的键的最后一个时间戳就成了t1b（因为map[A]里面已经没时间戳了，1步里已经清了），
    就清了 t1a～t1b 之间的数据
*/

sensor::TimedPointCloudOriginData RangeDataCollator::AddRangeData(
    const std::string& sensor_id,
    const sensor::TimedPointCloudData& timed_point_cloud_data) {
  CHECK_NE(expected_sensor_ids_.count(sensor_id), 0);
  // TODO(gaschler): These two cases can probably be one.
  if (id_to_pending_data_.count(sensor_id) != 0) {
    current_start_ = current_end_;
    // When we have two messages of the same sensor, move forward the older of
    // the two (do not send out current).
    current_end_ = id_to_pending_data_.at(sensor_id).time;
    auto result = CropAndMerge();
    id_to_pending_data_.emplace(sensor_id, timed_point_cloud_data);
    return result;
  }
  id_to_pending_data_.emplace(sensor_id, timed_point_cloud_data);
  if (expected_sensor_ids_.size() != id_to_pending_data_.size()) {
    return {};
  }
  current_start_ = current_end_;
  // We have messages from all sensors, move forward to oldest.
  common::Time oldest_timestamp = common::Time::max();
  for (const auto& pair : id_to_pending_data_) {
    oldest_timestamp = std::min(oldest_timestamp, pair.second.time);
  }
  current_end_ = oldest_timestamp;
  return CropAndMerge();
}

sensor::TimedPointCloudOriginData RangeDataCollator::CropAndMerge() {
  sensor::TimedPointCloudOriginData result{current_end_, {}, {}};
  bool warned_for_dropped_points = false;
  for (auto it = id_to_pending_data_.begin();
       it != id_to_pending_data_.end();) {
    sensor::TimedPointCloudData& data = it->second;
    sensor::TimedPointCloud& ranges = it->second.ranges;

    auto overlap_begin = ranges.begin();
    while (overlap_begin < ranges.end() &&
           data.time + common::FromSeconds((*overlap_begin)[3]) <
               current_start_) {
      ++overlap_begin;
    }
    auto overlap_end = overlap_begin;
    while (overlap_end < ranges.end() &&
           data.time + common::FromSeconds((*overlap_end)[3]) <= current_end_) {
      ++overlap_end;
    }
    if (ranges.begin() < overlap_begin && !warned_for_dropped_points) {
      LOG(WARNING) << "Dropped " << std::distance(ranges.begin(), overlap_begin)
                   << " earlier points.";
      warned_for_dropped_points = true;
    }

    // Copy overlapping range.
    if (overlap_begin < overlap_end) {
      std::size_t origin_index = result.origins.size();
      result.origins.push_back(data.origin);
      double time_correction = common::ToSeconds(data.time - current_end_);
      for (auto overlap_it = overlap_begin; overlap_it != overlap_end;
           ++overlap_it) {
        sensor::TimedPointCloudOriginData::RangeMeasurement point{*overlap_it,
                                                                  origin_index};
        // current_end_ + point_time[3]_after == in_timestamp +
        // point_time[3]_before
        point.point_time[3] += time_correction;
        result.ranges.push_back(point);
      }
    }

    // Drop buffered points until overlap_end.
    if (overlap_end == ranges.end()) {
      it = id_to_pending_data_.erase(it);
    } else if (overlap_end == ranges.begin()) {
      ++it;
    } else {
      data = sensor::TimedPointCloudData{
          data.time, data.origin,
          sensor::TimedPointCloud(overlap_end, ranges.end())};
      ++it;
    }
  }

  std::sort(result.ranges.begin(), result.ranges.end(),
            [](const sensor::TimedPointCloudOriginData::RangeMeasurement& a,
               const sensor::TimedPointCloudOriginData::RangeMeasurement& b) {
              return a.point_time[3] < b.point_time[3];
            });
  return result;
}

}  // namespace mapping
}  // namespace cartographer

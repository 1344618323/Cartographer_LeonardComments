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

#ifndef CARTOGRAPHER_SENSOR_TIMED_POINT_CLOUD_DATA_H_
#define CARTOGRAPHER_SENSOR_TIMED_POINT_CLOUD_DATA_H_

#include "Eigen/Core"
#include "cartographer/common/time.h"
#include "cartographer/sensor/point_cloud.h"

namespace cartographer {
namespace sensor {

//(cxn)这是存放某段雷达数据的结构
// 这段数据最后的时间戳
// 雷达相对baselink的坐标
// vector{雷达点在baselink系下坐标+时间戳}
struct TimedPointCloudData {
  common::Time time;
  Eigen::Vector3f origin;
  sensor::TimedPointCloud ranges;
};

//(cxn)多段雷达数据同步
// 这段数据最后的时间戳
// vector{雷达相对baselink的坐标}，比如有两个雷达，则vector的size为2
// vector{[雷达点坐标+时间戳],指着origins的索引}，比如某个雷达点是1号雷达测的，索引就是0
struct TimedPointCloudOriginData {
  struct RangeMeasurement {
    Eigen::Vector4f point_time;
    size_t origin_index;
  };
  common::Time time;
  std::vector<Eigen::Vector3f> origins;
  std::vector<RangeMeasurement> ranges;
};

// Converts 'timed_point_cloud_data' to a proto::TimedPointCloudData.
proto::TimedPointCloudData ToProto(
    const TimedPointCloudData& timed_point_cloud_data);

// Converts 'proto' to TimedPointCloudData.
TimedPointCloudData FromProto(const proto::TimedPointCloudData& proto);

}  // namespace sensor
}  // namespace cartographer

#endif  // CARTOGRAPHER_SENSOR_TIMED_POINT_CLOUD_DATA_H_

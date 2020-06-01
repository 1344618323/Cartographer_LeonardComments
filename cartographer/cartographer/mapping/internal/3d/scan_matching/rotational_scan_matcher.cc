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

#include "cartographer/mapping/internal/3d/scan_matching/rotational_scan_matcher.h"

#include <map>
#include <vector>

#include "cartographer/common/math.h"
#include "cartographer/common/port.h"

namespace cartographer {
namespace mapping {
namespace scan_matching {

/*(cxn)
rotational-scan-matcher 是为了给分枝定界scan matcher 减负用的，(以下内容可能有不太准确的地方，领会精神即可)
在2d中我们会对node的点云做预旋转，得到多组yaw旋转后的点云，但在3d中，这么搞代价太大，毕竟有三个旋转
因为IMU能算出 相对准确的 pitch与roll，在3d分枝定界中，我们只对yaw做对齐（cartographer的作者也在网站中强调3d中非常需要IMU）
但是因为有其他两个方向的旋转，直接像2d那样对点云那样做yaw旋转，然后与submap匹配算分是很难对上号的

所以搞了一个 rotational-scan-matcher

直方图具体算法
  输入以机器人位姿为坐标系的点云（去畸变后的），并左乘Rimu，得到 vector<point>
  将点云按z切分，z的步长为0.2m,得到 vector<vector<point>> slices
  for slice in slices:
    统计slice中的直方图向量
    for point in slice：
      有一参考点坐标a，当前点坐标b，slice质心坐标c，以 atan2(ba)×120/pi 为直方图横轴， bc与ba的夹角 为 直方图竖轴
    累积得到直方图向量hi，可以发现用了这个z方向的切分，我们就只会统计 落在同一平面上的点云 的内部结构关系了
  累积所有hi，最后输出一个120*1的直方图向量

在localslam中，每搞出一个node（就是能往submap中插点云的位姿），就会对那个node的点云算直方图
在globalslam中，每完成一个submap（submap中所有node的点云已算好了直方图），就累积成了一个submap的直方图，注意到这个submap的直方图要旋转至global系下

在3d的分枝定界中，我们也会对node的点云作预旋转，即通过论文公式6算出角度搜索步长，搞出一系列yaw旋转
对node的直方图做这些预旋转，再与submap的直方图向量求 余弦相似度，只有相似度高的yaw旋转，我们才对平移用分枝定界
*/


namespace {

constexpr float kMinDistance = 0.2f;
constexpr float kMaxDistance = 0.9f;
constexpr float kSliceHeight = 0.2f;

void AddValueToHistogram(float angle, const float value,
                         Eigen::VectorXf* histogram) {
  // Map the angle to [0, pi), i.e. a vector and its inverse are considered to
  // represent the same angle.
  while (angle > static_cast<float>(M_PI)) {
    angle -= static_cast<float>(M_PI);
  }
  while (angle < 0.f) {
    angle += static_cast<float>(M_PI);
  }
  const float zero_to_one = angle / static_cast<float>(M_PI);
  const int bucket = common::Clamp<int>(
      common::RoundToInt(histogram->size() * zero_to_one - 0.5f), 0,
      histogram->size() - 1);
  (*histogram)(bucket) += value;
}

Eigen::Vector3f ComputeCentroid(const sensor::PointCloud& slice) {
  CHECK(!slice.empty());
  Eigen::Vector3f sum = Eigen::Vector3f::Zero();
  for (const Eigen::Vector3f& point : slice) {
    sum += point;
  }
  return sum / static_cast<float>(slice.size());
}

void AddPointCloudSliceToHistogram(const sensor::PointCloud& slice,
                                   Eigen::VectorXf* const histogram) {
  if (slice.empty()) {
    return;
  }
  // We compute the angle of the ray from a point to the centroid of the whole
  // point cloud. If it is orthogonal to the angle we compute between points, we
  // will add the angle between points to the histogram with the maximum weight.
  // This is to reject, e.g., the angles observed on the ceiling and floor.
  const Eigen::Vector3f centroid = ComputeCentroid(slice);
  Eigen::Vector3f last_point = slice.front();
  for (const Eigen::Vector3f& point : slice) {
    const Eigen::Vector2f delta = (point - last_point).head<2>();
    const Eigen::Vector2f direction = (point - centroid).head<2>();
    const float distance = delta.norm();

    //(cxn)当前点到参考点的距离太小 或者 当前点到质心的距离太小（小于0.2m）就跳过
    if (distance < kMinDistance || direction.norm() < kMinDistance) {
      continue;
    }
    // 若当前点到参考点的距离较远时（0.9m），更换参考点
    if (distance > kMaxDistance) {
      last_point = point;
      continue;
    }
    // 求当前点相对于参考点的角度
    const float angle = common::atan2(delta);
    // 权值可以这么理解：求当前点b到质心c的向量bc 与 当前点b到参考点a的向量ba 的夹角 
    // 若接近90度，说明ab可能是一个垂直于激光线bc的平面，这时我们认为探测结果应该是比较准确的，权重高；
    // 相反，如果ab与bc近乎平行时，说明激光线bc近乎平行于障碍物，这时可能探测误差较大（可能由传感器引起，也有可能是孔洞、玻璃等），因此权重应该降低。
    // 直方图将 [0, pi) 分成120份，权值是累加的，要注意这些切片得到的点云组 用的是 同一个直方图对象
    const float value = std::max(
        0.f, 1.f - std::abs(delta.normalized().dot(direction.normalized())));
    AddValueToHistogram(angle, value, histogram);
  }
}

// A function to sort the points in each slice by angle around the centroid.
// This is because the returns from different rangefinders are interleaved in
// the data.
// (cxn)按角度从小到大：(-pai,+pai]
sensor::PointCloud SortSlice(const sensor::PointCloud& slice) {
  struct SortableAnglePointPair {
    bool operator<(const SortableAnglePointPair& rhs) const {
      return angle < rhs.angle;
    }

    float angle;
    Eigen::Vector3f point;
  };
  const Eigen::Vector3f centroid = ComputeCentroid(slice);
  std::vector<SortableAnglePointPair> by_angle;
  by_angle.reserve(slice.size());
  for (const Eigen::Vector3f& point : slice) {
    const Eigen::Vector2f delta = (point - centroid).head<2>();
    if (delta.norm() < kMinDistance) {
      continue;
    }
    by_angle.push_back(SortableAnglePointPair{common::atan2(delta), point});
  }
  std::sort(by_angle.begin(), by_angle.end());
  sensor::PointCloud result;
  for (const auto& pair : by_angle) {
    result.push_back(pair.point);
  }
  return result;
}

// Rotates the given 'histogram' by the given 'angle'. This might lead to
// rotations of a fractional bucket which is handled by linearly interpolating.
/*(cxn) 旋转直方图
  假设对于这个node，其 yaw(Rglobal-imu)=-30度，那么对于global坐标系顺时针转30度，可以得到imu坐标系
    rotate-by-buckets= 30/180*120; full-buckets=19
    也就是说  global坐标系下的直方图的第1个bin 应该是 imu坐标系下的直方图的第20个bin与第21个bin插值得到的
  相反的， 若其 yaw(Rglobal-imu)=30度，那么对于global坐标系逆时针转30度，可以得到imu坐标系
    rotate-by-buckets= -30/180*120; full-buckets=-21+120=99
    也就是说  global坐标系下的直方图的第1个bin 应该是 imu坐标系下的直方图的第99个bin与第100个bin插值得到的
    global坐标系下的直方图的第21个bin 应该是 imu坐标系下的直方图的第(21+99)%120个bin与第(22+99)%120=1个bin插值得到的
*/
Eigen::VectorXf RotateHistogram(const Eigen::VectorXf& histogram,
                                const float angle) {
  const float rotate_by_buckets = -angle * histogram.size() / M_PI;
  // 减去0.5变成向下取整
  int full_buckets = common::RoundToInt(rotate_by_buckets - 0.5f);
  // 总是一个小于1大于0的数
  const float fraction = rotate_by_buckets - full_buckets;

  while (full_buckets < 0) {
    full_buckets += histogram.size();
  }

  Eigen::VectorXf rotated_histogram_0 = Eigen::VectorXf::Zero(histogram.size());
  Eigen::VectorXf rotated_histogram_1 = Eigen::VectorXf::Zero(histogram.size());
  for (int i = 0; i != histogram.size(); ++i) {
    rotated_histogram_0[i] = histogram[(i + full_buckets) % histogram.size()];
    rotated_histogram_1[i] =
        histogram[(i + 1 + full_buckets) % histogram.size()];
  }
  // 线性插值
  return fraction * rotated_histogram_1 +
         (1.f - fraction) * rotated_histogram_0;
}


float MatchHistograms(const Eigen::VectorXf& submap_histogram,
                      const Eigen::VectorXf& scan_histogram) {
  // We compute the dot product of normalized histograms as a measure of
  // similarity.
  const float scan_histogram_norm = scan_histogram.norm();
  const float submap_histogram_norm = submap_histogram.norm();
  const float normalization = scan_histogram_norm * submap_histogram_norm;
  if (normalization < 1e-3f) {
    return 1.f;
  }
  return submap_histogram.dot(scan_histogram) / normalization;
}

}  // namespace

Eigen::VectorXf RotationalScanMatcher::ComputeHistogram(
    const sensor::PointCloud& point_cloud, const int histogram_size) {
  //(cxn)默认分成120组
  Eigen::VectorXf histogram = Eigen::VectorXf::Zero(histogram_size);
  // 通过z切分，步长为0.2m
  std::map<int, sensor::PointCloud> slices;
  for (const Eigen::Vector3f& point : point_cloud) {
    slices[common::RoundToInt(point.z() / kSliceHeight)].push_back(point);
  }
  for (const auto& slice : slices) {
    // 对切分的每一组点云：按 到点云质心的距离 从小到大 进行排序，然后再统计直方图 
    AddPointCloudSliceToHistogram(SortSlice(slice.second), &histogram);
  }
  return histogram;
}

/*(cxn)
对该submap内所有node的直方图求和，细想下非常合理
例如在无移动物体的环境中得到的一系列点云，在imu坐标系下坐标b (xi,yi)，参考点坐标a(x1,y1)， 其质心c为(xhat,yhat)
若车子移动了(deltax,deltay),则这些点云的坐标b'变成 (xi-deltax,yi-deltay)，参考点坐标a'(x1-deltax,y1-deltay)，其质心c'为(xhat-deltax,yhat-detlay)
计算直方图的过程中的用的变量 atan2(ba) = atan2(b'a')；ba 与 bc 的内积 = b'a' 与 b'c' 的内积
也就说，理想情况下，在同一个submap中的node的直方图应该是一样的
而这里的求和理想情况下应该是这些同样的直方图的累加，最终后的直方图向量方向没有改变
*/
RotationalScanMatcher::RotationalScanMatcher(
    const std::vector<std::pair<Eigen::VectorXf, float>>& histograms_at_angles)
    : histogram_(
          Eigen::VectorXf::Zero(histograms_at_angles.at(0).first.size())) {
  for (const auto& histogram_at_angle : histograms_at_angles) {
    histogram_ +=
        RotateHistogram(histogram_at_angle.first, histogram_at_angle.second);
  }
}

// 余弦相似度，就是计算两个向量间的夹角的余弦值；余弦距离就是用1减去这个获得的余弦相似度。
// 这个函数就是对某个node的直方图 进行不同的旋转，并与submap的直方图计算 余弦相似度 作为得分
std::vector<float> RotationalScanMatcher::Match(
    const Eigen::VectorXf& histogram, const float initial_angle,
    const std::vector<float>& angles) const {
  std::vector<float> result;
  result.reserve(angles.size());
  for (const float angle : angles) {
    const Eigen::VectorXf scan_histogram =
        RotateHistogram(histogram, initial_angle + angle);
    result.push_back(MatchHistograms(histogram_, scan_histogram));
  }
  return result;
}

}  // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer

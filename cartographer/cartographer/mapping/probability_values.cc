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

#include "cartographer/mapping/probability_values.h"

#include "cartographer/common/make_unique.h"

namespace cartographer {
namespace mapping {

namespace {

// 0 is unknown, [1, 32767] maps to [lower_bound, upper_bound].
float SlowValueToBoundedFloat(const uint16 value, const uint16 unknown_value,
                              const float unknown_result,
                              const float lower_bound,
                              const float upper_bound) {
  CHECK_GE(value, 0);
  CHECK_LE(value, 32767);
  if (value == unknown_value) return unknown_result;
  const float kScale = (upper_bound - lower_bound) / 32766.f;
  return value * kScale + (lower_bound - kScale);
}

std::unique_ptr<std::vector<float>> PrecomputeValueToBoundedFloat(
    const uint16 unknown_value, const float unknown_result,
    const float lower_bound, const float upper_bound) {
  auto result = common::make_unique<std::vector<float>>();
  // Repeat two times, so that both values with and without the update marker
  // can be converted to a probability.
  for (int repeat = 0; repeat != 2; ++repeat) {
    for (int value = 0; value != 32768; ++value) {
      //(cxn)将 unknown_value投影为unknown_result，将1~32767 投影到lower_bound到upper_bound
      //可以是 将 0 投 成0;1~32767 投到 0.1~0.9
      // 结果重复两次
      result->push_back(SlowValueToBoundedFloat(
          value, unknown_value, unknown_result, lower_bound, upper_bound));
    }
  }
  return result;
}

std::unique_ptr<std::vector<float>> PrecomputeValueToProbability() {
  return PrecomputeValueToBoundedFloat(kUnknownProbabilityValue,
                                       kMinProbability, kMinProbability,
                                       kMaxProbability);
}

//(cxn)将 0 投 成0;1~32767 投到 0.1~0.9,重复两次
std::unique_ptr<std::vector<float>> PrecomputeValueToCorrespondenceCost() {
  return PrecomputeValueToBoundedFloat(
    //(cxn) 0 0.9;0.1;0.9
      kUnknownCorrespondenceValue, kMaxCorrespondenceCost,
      kMinCorrespondenceCost, kMaxCorrespondenceCost);
}

}  // namespace

const std::vector<float>* const kValueToProbability =
    PrecomputeValueToProbability().release();


//(cxn)返回 的数组是 将 0 投 成0;1~32767 投到 0.1~0.9,重复两次
// 用于快速查表，如 查询 32768*2-1 对应的值为 0.9
const std::vector<float>* const kValueToCorrespondenceCost =
    PrecomputeValueToCorrespondenceCost().release();

std::vector<uint16> ComputeLookupTableToApplyOdds(const float odds) {
  std::vector<uint16> result;
  result.push_back(ProbabilityToValue(ProbabilityFromOdds(odds)) +
                   kUpdateMarker);
  for (int cell = 1; cell != 32768; ++cell) {
    result.push_back(ProbabilityToValue(ProbabilityFromOdds(
                         odds * Odds((*kValueToProbability)[cell]))) +
                     kUpdateMarker);
  }
  return result;
}

/*(cxn)
 给odd(s|z)的结果创建一个表，能够快速查出 phit或者pfree对 {网格是障碍物的成本值} 的修正结果 
 这里我们偷个懒，odd(s=1)=P(s=1)/P(s=0)=P(s=1)/(1-P(s=1))，将odd(s=1)简写成odd(s)
 例如 phit=0.55，输入参数就是 odds = [p(z=1|s=1)/p(z=1|s=0)]=0.55/0.45
 此时，对于 未知网格 返回 s(0.45)+32768；对于 已知网格 其返回值 会 降低
 而对于 pfree=0.49，输入参数就是 odds = [p(z=0|s=1)/p(z=0|s=0)]=0.49/0.51
 此时，对于 未知网格 返回 s(0.51)+32768；对于 已知网格 其返回值 会 升高
*/
std::vector<uint16> ComputeLookupTableToApplyCorrespondenceCostOdds(
    float odds) {
  std::vector<uint16> result;
  //(cxn)根据odd，求p，并将 1-p 缩放到 [1,32767]
  //result 存入 Sp+32768 用于对应栅格 未知的情况
  result.push_back(CorrespondenceCostToValue(ProbabilityToCorrespondenceCost(
                       ProbabilityFromOdds(odds))) +
                   kUpdateMarker);
  for (int cell = 1; cell != 32768; ++cell) {
    result.push_back(
      //(cxn) 将成本值缩放到 (1～32767)+32768
        CorrespondenceCostToValue(
          //栅格是障碍物的成本：1-P(s=1)
            ProbabilityToCorrespondenceCost(ProbabilityFromOdds(//odd(s)=P(s=1)/P(s=0)=P(s=1)/(1-P(s=1)),求P(s=1)
              //即 odd(s|z)=[p(z|s=1)/p(z|s=0)]*odd(s)，算出从 odd(s)从0.9到0.1的所有情况
              // 输入参数odds 就是 [p(z|s=1)/p(z|s=0)]，有两种情况 [p(z=1|s=1)/p(z=1|s=0)]，or [p(z=0|s=1)/p(z=0|s=0)]
                odds * Odds(CorrespondenceCostToProbability(//返回栅格是障碍物的概率：0.9~0.1
                  //返回栅格是障碍物的成本： 0.1~0.9 若是障碍物应该返回0.1
                           (*kValueToCorrespondenceCost)[cell]))))) +
        kUpdateMarker);
  }
  return result;
}

}  // namespace mapping
}  // namespace cartographer

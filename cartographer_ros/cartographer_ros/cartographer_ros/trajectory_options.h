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

#ifndef CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_TRAJECTORY_OPTIONS_H
#define CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_TRAJECTORY_OPTIONS_H

#include <string>

#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/port.h"
#include "cartographer/mapping/proto/trajectory_builder_options.pb.h"
#include "cartographer_ros_msgs/TrajectoryOptions.h"

namespace cartographer_ros {

struct TrajectoryOptions {
  ::cartographer::mapping::proto::TrajectoryBuilderOptions
      trajectory_builder_options;
  std::string tracking_frame;
  std::string published_frame;
  std::string odom_frame;
  bool provide_odom_frame;
  bool use_odometry;
  bool use_nav_sat;
  bool use_landmarks;
  bool publish_frame_projected_to_2d;
  int num_laser_scans;
  int num_multi_echo_laser_scans;
  int num_subdivisions_per_laser_scan;
  int num_point_clouds;
  double rangefinder_sampling_ratio;
  double odometry_sampling_ratio;
  double fixed_frame_pose_sampling_ratio;
  double imu_sampling_ratio;
  double landmarks_sampling_ratio;
  
  //(cxn)离线2d中的配置参数
    /*
    trajectory_builder = TRAJECTORY_BUILDER,
    tracking_frame = "base_link",
    published_frame = "base_link",
    odom_frame = "odom",
    provide_odom_frame = true,
    publish_frame_projected_to_2d = false,
    use_odometry = false,
    use_nav_sat = false,
    use_landmarks = false,
    num_laser_scans = 0,
    num_multi_echo_laser_scans = 1,
    num_subdivisions_per_laser_scan = 10,
    num_point_clouds = 0,
    rangefinder_sampling_ratio = 1.,
    odometry_sampling_ratio = 1.,
    fixed_frame_pose_sampling_ratio = 1.,
    imu_sampling_ratio = 1.,
    landmarks_sampling_ratio = 1.,
  */
};

::cartographer::mapping::proto::InitialTrajectoryPose
CreateInitialTrajectoryPose(
    ::cartographer::common::LuaParameterDictionary* lua_parameter_dictionary);

TrajectoryOptions CreateTrajectoryOptions(
    ::cartographer::common::LuaParameterDictionary* lua_parameter_dictionary);

TrajectoryOptions CreateTrajectoryOptions(
    ::cartographer::common::LuaParameterDictionary* lua_parameter_dictionary,
    ::cartographer::common::LuaParameterDictionary* initial_trajectory_pose);

// Try to convert 'msg' into 'options'. Returns false on failure.
bool FromRosMessage(const cartographer_ros_msgs::TrajectoryOptions& msg,
                    TrajectoryOptions* options);

// Converts 'trajectory_options' into a ROS message.
cartographer_ros_msgs::TrajectoryOptions ToRosMessage(
    const TrajectoryOptions& trajectory_options);

}  // namespace cartographer_ros

#endif  // CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_TRAJECTORY_OPTIONS_H

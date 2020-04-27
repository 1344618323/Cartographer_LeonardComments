# Leonard Chen 的注释

主要注释了源码中关于 2D SLAM 的部分，3D部分有空再学习吧。

再次膜拜写出这套代码的大佬们！

<table><td bgcolor=yellow><font color=black>一点思考：Cartographer在检测回环的时候，其实就是只要一帧数据与submap匹配上了就是回环，个人感觉这里可以加一个连续性检测，就像ORB-SLAM一样，连续几帧与同一个submap匹配上了，我才算一个回环，会不会更鲁棒一些呢？</font></td></tr></table>

以下是学习过程中随手写的笔记，比较乱。

## LOCAL 2D SLAM
```
输入雷达数据->多雷达数据同步->使用外推器推算每个激光束时刻 i 的机器人在local-frame中的位姿 Tlw-i（即 去畸变），对点云数据有 Tlw-i*pi

变量gravity_alignment就是外推器中imu-tracker-推出来的imu中的旋转，在cartographer中，就认为其roll和pitch基本没错，yaw有问题

搜集够一定的激光束后，time时刻（就是最后一个激光束的时刻），就准备scan-match优化了

对所有激光数据有 gravity_alignment(time)*(Tlw-time’*Tlw-i)*pi

Tlw-time’*Tlw-i 是以最后一激光测量时刻time的位姿为坐标系的，这样就得到的无畸变的激光测量数据 (Tlw-time’*Tlw-i)*pi

左乘 gravity_alignment(time) 是为了 将数据投影到2d，因为cartographer本来是背包上建图设备运行的，会有pitch与roll角，造成激光点不在一个平面上，
这样处理之后，点云（x，y，z）中的（x，y）就可以看做是一个平面上的坐标了
可以发现这么做还是多出了一个yaw的旋转，没事优化之后会变回去
这里要对点云做一次体素滤波，存到 gravity-aligned-range-data = filter{ gravity_alignment(time)*(Tlw-time’*Tlw-i)*pi}

scan-match初值为2d变换： F3d->2d(Tlw-time * gravity_alignment(time)’)，scan-match之前还要搞一次自适应体素滤波

优化后，就得到了估计位姿  Tlw-time = F2d->3d(pose-estimate-2d) * gravity_alignment(time)
更新外推器，调用外推器的函数 AddPose(Tlw-time)

更新点云在local-frame中的坐标 range-data-in-local = F2d->3d(pose-estimate-2d)*gravity-aligned-range-data

将这些点云 range-data-in-local 加入submap2D中，只用三维向量的前两维
对 gravity-aligned-range-data 执行自适应体素滤波（比scan-match滤得更凶，也就是说插submap的点云比回环检测的点云更稠密些）准备用于回环，得到 filtered_gravity_aligned_point_cloud


如果 time 时刻，发现要新建的submap，其原点就是 Tlw-time（2D）


只要通过了motion-filter就往submap里插点云，返回如下结果，否则返回nullptr
InsertionResult
  TrajectoryNode::Data
    time,
    gravity_alignment,
    filtered_gravity_aligned_point_cloud

    pose_estimate  就是Tlw-time（3D）
  
  insertion_submaps 被插的两个submap


local-frame返回结果

MatchingResult
  time, 
  pose_estimate, 
  range_data_in_local,  （未经自适应滤波的）
  insertion_result
```
---


## POSE EXTRAPOLATOR

```
这个类其实就是用来推算机器人在local_frame中的位姿的
local_frame在cartographer中的意思就是未经过回环检测与pose-graph优化的坐标系

核心函数 ExtrapolatePose(time)就是推算 time 时刻机器人位姿的

每次local—trajectory-bulider中经过scan-match优化后，会将优化后的位姿（3D的）加入到timed_pose_queue_中（即local—trajectory-bulider 调用函数 addpose）
timed_pose_queue_.back() 就是 最新的优化结果，我们记为 Tlw-t1

然后 ExtrapolatePose(time) 用来 在 Tlw-t1的基础上计算 Tlw-t2

每次 Tlw-t1 加入之后，会计算 (Tlw-t0'*Tlw-t1) ->  线速度 V = t(t1)-t(t0)/(t1-t0)  角速度 w = 转成旋转向量(Rlw-t0'*Rlw-t1) /(t1-t0)
odom也会算 线速度与角速度，待会再说

同时这里有个对象，Imu-tracker-用来一直估计 机器人的旋转， 我们可以认为这个传感器的坐标系为 imu-frame，其估计的旋转值为 Rimuinit-t吧
在没有IMU数据的时候，就用上面推算的角速度w 模拟 真实传感器IMU的输入，同时用 (1,0,0)*10 模拟真实传感器加速度计的数值
Imu-tracker-的两种更新情况：   
    A. 每次 local—trajectory-bulider 调用函数 addpose时更新
    B. 每次 local—trajectory-bulider 调用函数 EstimateGravityOrientation 时更新 （就是搜集好雷达数据之后，准备scan-match优化之前）

可以发现在local—trajectory-bulider-2d中，cartographer是十分信任 Imu-tracker-的旋转的（除了yaw），
每次scan-match优化后，会将优化输出的2D变换矩阵T2d 操作以 {F2d->3d(T2d)}*R(imu)，作为加入到timed_pose_queue_中位姿
还不知道3d中是怎么搞的


说下ExtrapolatePose(time)怎么做的吧：
已知Tlw-t1=[R(t1) t(t1)]
要算Tlw-t1=[R(t2) t(t2)]

1. t(t2)= t(t1) + v(t2-t1)
2. 使用对象 extrapolation-imu-tracker- 来推算角度
    extrapolation-imu-tracker- 在每次 Imu-tracker-更新后再复制，
    这样用 extrapolation-imu-tracker-就能先更新到时刻t2，而不影响Imu-tracker-保持在t1的状态了
    此时，extrapolation-imu-tracker-算出的旋转为 Rimuinit-t2，而Imu-tracker-的状态为Rimuinit-t1

    R(t2)= R(t1)* (Rimuinit-t1)'*(Rimuinit-t2)


最后说一下odom是怎么估计线速度与角速度的吧
已知在 odom-frame 坐标系下，有one、two两个时刻的位姿数据：Todom-one，Todom-two，two是最新的时刻
时刻差为 deltat=one-two
有 Todom-two'*Todom-one=[ R(two)'R(one)  R(two)'(t(one)-t(two))]
1. 角速度无疑问就是 w = 转成旋转向量(R(two)'R(one)) /deltat
2. 线速度有个问题就是要 求位移差(t(one)-t(two))  在 lcaol-frame中的坐标，
   (t(one)-t(two))是odo-frame中坐标，应写成 t(odom-one)-t(odom-two)
    使用对象 odometry-imu-tracker- 来推算角度
    odometry-imu-tracker- 也是在每次 Imu-tracker-更新后再复制，与extrapolation-imu-tracker-同理， 
    two时刻，odometry-imu-tracker-算出的旋转为 Rimuinit-two，而Imu-tracker-的状态为Rimuinit-t1
    可以推算出机器人在two时刻时，在local-frame中的旋转：     Rlw-two= Rlw-t1* (Rimuinit-t1)'*(Rimuinit-two)

  local-frame到 odom-frame的变换为 Tlw-odom,其中tlw-dom的值我们不用管，因为 (t(one)-t(two))是个差，变换后关于 tlw-dom 的部分，自然会被消掉
  也就说  Tlw-odom*(t(odom-one)-t(odom-two))= Rlw-odom*t(odom-one)-t(odom-two)
  Rlw-odom= Rlw-two* R(two)'
  所以，线速度为 {Rlw-two* R(two)'*(t(one)-t(two))}/deltat
```
---

## GLOBAL 2D SLAM
```
可以看出来，cartographer代码还没彻底完成，不过官方好像有一段时间没更了。。。

一切都是为了最后的 Sparse pose adjustment （SPA）

在localslam中对往submap里插点云的node，会插入到 pose-graph 中

插入node时执行：
  trajectory-nodes-加入新node、往submap-data-加入新submpa（如果有的话）
  并给 constraint bulider 发布任务：给node和submap计算约束


计算约束：
  给优化问题新增submap，主要设置新增submap的全局位姿的初始值，会优化的
  用优化问题中的解出的最新的submap的全局位姿来推理：Tgw_submap2 = Tgw_submap1 * Tlw_submap1' * Tlw_submap2

  给优化问题加入这个新node，并设置属性：
    1.node在local-frame中2d位姿：不变的 Tlw_node_2d
    2.node在global-frame中2d位姿：设置一个初值，会优化的。
      初始值用该node所在submap在优化问题中优化值来推理：Tgw_node_2d = Tgw_submap1 * Tlw_submap1' * Tlw_node_2d
    3.node在imu-frame中的旋转：不变的 Rimu_node

    注意：Tlw_node_2d= F3d->2d(Tlw_node*Rimu_node'),这就是localslam中ceres优化的结果（注意此时是少了一个IMU中的yaw角的）
  
  
  设置 node 与 submap（localslam输入的两个或一个） 的联系
  优化问题新增约束：即优化中边的观测值 Tlw-submap‘*Tlw-node-2d
    * spa公式： Tij‘ * Ti' *Tj
    * 在cartographer中，Tsubmap-node_2d就是 Tij，就是边的观测值
    * Ti就是 Tgw-submap；Tj就是 Tgw-node    

  分枝定界找约束：
    从所有完成的submap（包括不同轨迹的）中，找新node与submap约束，另外这些已完成的submap一定没有当前的node
    找一下新完成的submap与之前的node（包括不同轨迹的，但不包括源生此submap的node）的约束

  往优化问题中插入足够多的node之后，就SPA（2d.lua中设置为每90个node，而纯定位中设置为每20个node）


分枝定界找node与submap的约束：
    给分枝定界scan-match设定初值，初值为 Tinit = Tgw_submap'*Tgw_node_2d
    在初始化scan-matcher时，需要对submap多分辨率处理，且后面的扫描匹配也很耗时，所以要多任务处理（多任务调度怎么实现，还没仔细瞧过）

    点云坐标是 Rimu_node*Pnode （注意这个点云在localslam中是经过为回环配置的自适应体素滤波器滤波过的）

    A.若submap与node在同一轨迹上 or submap的轨迹与node的轨迹在不久前刚刚绑定，则用局部匹配：
        Tinit.translate() 在要在15m以内，否则退出

        为了减少计算量，加一个抽样器，如3次，才执行一次：
            从初值出发分枝定界匹配，搜索窗口较小 -7m～7m，-30°～30°
       
    B.否则 全局匹配：
        为了减少计算量，加一个抽样器，如300次，才执行一次：
            从submap中心出发分枝定界匹配，搜索窗口较大 -5e4m～5e4m(这个值会根据node的点云处理)，-180°～180°
    
    若找到了回环，就用 ceres scan match 再优化下

    优化问题新增约束：即优化中边的观测值 Tb&b



SPA优化：
  优化所有node与submap（固定的除外，且第一个submap（如果A轨迹有s1，s2，B轨迹有s3，s4，s5，按时间排序，第一个submap就是s1）也会固定）
  
  固定的submap与node会加入优化问题，但不优化

  对每个submap与node有
    使用localslam中的观测值：(Tlw-submap‘*Tlw-node-2d)‘ * Tgw_submap' * Tgw_node_2d
    使用回环检测中的观测值： Tb&b * Tgw_submap' * Tgw_node_2d

  对每条轨迹中相邻的node有 
    使用localslam中的观测值：(Tlw_node1_2d'Tlw_node2_2d) * Tgw_node_2d' * Tgw_node_2d
    使用odom的观测值： 用插值得到这两个node时刻在odom中的位姿 Todom_node1,Todom_node2，此外还要左乘 Rimu_node1,Rimu_node2
                 (Todom_node1_2d'Todom_node2_2d) * Tgw_node_2d' * Tgw_node_2d

  优化后，将优化结果传播出去： 
    优化后的node有 Tgw_node = Tgw_node_2d * Rimu_node 
    并将优化进行时，新加入的node的位姿也就行修改： Tgw_submap* Toldgw_submap' * Toldgw_node



关于纯定位：
  纯定位时，会将pbstream中包含的轨迹（node & submap）全部加载到pose-graph中，且这些node、submap是在优化问题中固定值

  运行纯定位时，是重新开了一条轨迹A

  localslam的步骤与slam的完全一样

  globalslam的步骤与slam的大部分相同，区别在于：由于 轨迹A 与之前的估计不同，会先进入全局的分枝定界搜索，后面可能会进入局部的分枝定界搜索

  每次SPA优化之后，会对 轨迹A 中的submap缩减，减到3个以内
```

---


## cartographer_ros中
这里大概写下ros中传感器数据是怎么调度到cartgrapher中的吧，写得比较乱，看得也不太明白

```
读取配置 node_options, trajectory_options

新建MapBuilder对象
map_builder = cartographer::mapping::MapBuilder(node_options.map_builder_options);
    线程池：        common::ThreadPool thread_pool_ 
    pose-graph：   (PoseGraph2D or PoseGraph3D) pose_graph_ 建立时传参 thread_pool_
    传感器数据搜集： (TrajectoryCollator or Collator) sensor_collator_ 


新建Node对象
node = Node(node_options, std::move(map_builder), &tf_buffer)
    MapBuilderBridge map_builder_bridge_(node_options_, std::move(map_builder), tf_buffer)

    配置发送的topic以及service


node.StartTrajectoryWithDefaultTopics(trajectory_options)
    Node::AddTrajectory(trajectory_options)
        map_builder_bridge_.AddTrajectory(expected_sensor_ids, options)获取新轨迹编号
            vector<CollatedTrajectoryBuilder> trajectory_builders_

            trajectory_builders_.push_back(
                CollatedTrajectoryBuilder(
                    sensor_collator_, trajectory_id, expected_sensor_ids,
                    CreateGlobalTrajectoryBuilder2D(std::move(local_trajectory_builder), trajectory_id,pose_graph_),
                        local_slam_result_callback))
                总之里面有对 sensor_collator_ 操作
                将每个传感器的回调函数设置为 HandleCollatedSensorData

        给当前轨迹配置一个外推器，加入 std::map<int, ::cartographer::mapping::PoseExtrapolator> extrapolators_;
        给当前轨迹配置一个采样器，加入 std::unordered_map<int, TrajectorySensorSamplers> sensor_samplers_;


对每条轨迹都有
    local_trajectory_bulider_2d
        active_submaps_(options.submaps_options()),
        motion_filter_(options_.motion_filter_options()),
        real_time_correlative_scan_matcher_(options_.real_time_correlative_scan_matcher_options()),
        ceres_scan_matcher_(options_.ceres_scan_matcher_options()),
        range_data_collator_(expected_range_sensor_ids) {}

    GlobalTrajectoryBuilder(
        trajectory_id_(trajectory_id),
        pose_graph_(pose_graph),
        local_trajectory_builder_(std::move(local_trajectory_builder)),
        local_slam_result_callback_(local_slam_result_callback) {}

    CollatedTrajectoryBuilder::CollatedTrajectoryBuilder(
        sensor_collator_(sensor_collator),
        trajectory_id_(trajectory_id),
        wrapped_trajectory_builder_(std::move(wrapped_trajectory_builder)), 就是一个 GlobalTrajectoryBuilder
        last_logging_time_(std::chrono::steady_clock::now())

map_bulider_的  std::vector<std::unique_ptr<mapping::TrajectoryBuilderInterface>> trajectory_builders_; 
存到就是每条轨迹的CollatedTrajectoryBuilder


对每条轨迹都有
SensorBridge::SensorBridge(
    num_subdivisions_per_laser_scan_(num_subdivisions_per_laser_scan),
    tf_bridge_(tracking_frame, lookup_transform_timeout_sec, tf_buffer),
    trajectory_builder_(trajectory_builder) 也是CollatedTrajectoryBuilder

ros传感器数据回调
    |
    sensor_bridge_->HandleXXXXMessage
    一些预处理，如雷达是分段与时间戳
    |
    sensor_bridge_.trajectory_builder_->AddSensorData
        |
        也就是 CollatedTrajectoryBuilder
        sensor_collator_->AddSensorData
            |
            也就是 Collator::AddSensorData

            传感器的数据会被压成
            template <typename DataType>
            class Dispatchable : public Data
            Dispatchable(const std::string &sensor_id, const DataType &data)

            这个函数还做了与Collator中消息队列有关的操作（没搞明白）
                |
                data->callback
                也就是 Dispatchable::AddToTrajectoryBuilder(trajectory_builder)
                    |  
                    这个trajectory_builder是GlobalTrajectoryBuilder对象
                    GlobalTrajectoryBuilder：：AddSensorData
                        |
                        for IMU or Odom
                        local_trajectory_builder_->AddXXXXData
                        pose_graph_->AddXXXXData
                            |
                            将任务放到 work_queue_中
                            任务：OptimizationProblem2D->AddXXXData
                                |
                                xxx_data_.Append(轨迹id，数据)

                        for Laser
                        local_trajectory_builder_->AddXXXXData
                        pose_graph_->AddNode
```
---

## 配置文件

配置文件太多了，跳来跳去的奇烦，放到这里方面对照

看下backup_2d.lua的配置

node_options中的内容
```
map_builder = MAP_BUILDER,
    MAP_BUILDER（cartographer/configuration_files/map_builder.lua）默认值为
    |
    use_trajectory_builder_2d = false,(2d中这里会改成true)
    use_trajectory_builder_3d = false,
    num_background_threads = 4,
    pose_graph = POSE_GRAPH,
        POSE_GRAPH（cartographer/configuration_files/pose_graph.lua）2d用的是默认值
        |
        optimize_every_n_nodes = 90,
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
        matcher_translation_weight = 5e2,
        matcher_rotation_weight = 1.6e3,
        optimization_problem = {
            huber_scale = 1e1,
            acceleration_weight = 1e3,
            rotation_weight = 3e5,
            local_slam_pose_translation_weight = 1e5,
            local_slam_pose_rotation_weight = 1e5,
            odometry_translation_weight = 1e5,
            odometry_rotation_weight = 1e5,
            fixed_frame_pose_translation_weight = 1e1,
            fixed_frame_pose_rotation_weight = 1e2,
            log_solver_summary = false,
            ceres_solver_options = {
            use_nonmonotonic_steps = false,
            max_num_iterations = 50,
            num_threads = 7,
            },
        },
        max_num_final_iterations = 200,
        global_sampling_ratio = 0.003,
        log_residual_histograms = true,
        global_constraint_search_after_n_seconds = 10.,
      
    MAP_BUILDER.use_trajectory_builder_2d = true

map_frame = "map",
lookup_transform_timeout_sec = 0.2,
submap_publish_period_sec = 0.3,
pose_publish_period_sec = 5e-3,
trajectory_publish_period_sec = 30e-3,
```

trajectory_option
```
trajectory_builder = TRAJECTORY_BUILDER,
    默认值（cartographer/configuration_files/trajectory_builder.lua）
    |
    include "trajectory_builder_2d.lua"
    include "trajectory_builder_3d.lua"

    TRAJECTORY_BUILDER = {
        trajectory_builder_2d = TRAJECTORY_BUILDER_2D,
            |
            TRAJECTORY_BUILDER_2D = {
                use_imu_data = true,
                min_range = 0.,
                max_range = 30.,
                min_z = -0.8,
                max_z = 2.,
                missing_data_ray_length = 5.,
                num_accumulated_range_data = 1,(backup2d.lua中改成了10)
                voxel_filter_size = 0.025,

                adaptive_voxel_filter = {
                    max_length = 0.5,
                    min_num_points = 200,
                    max_range = 50.,
                },

                loop_closure_adaptive_voxel_filter = {
                    max_length = 0.9,
                    min_num_points = 100,
                    max_range = 50.,
                },

                use_online_correlative_scan_matching = false,
                real_time_correlative_scan_matcher = {
                    linear_search_window = 0.1,
                    angular_search_window = math.rad(20.),
                    translation_delta_cost_weight = 1e-1,
                    rotation_delta_cost_weight = 1e-1,
                },

                ceres_scan_matcher = {
                    occupied_space_weight = 1.,
                    translation_weight = 10.,
                    rotation_weight = 40.,
                    ceres_solver_options = {
                    use_nonmonotonic_steps = false,
                    max_num_iterations = 20,
                    num_threads = 1,
                    },
                },

                motion_filter = {
                    max_time_seconds = 5.,
                    max_distance_meters = 0.2,
                    max_angle_radians = math.rad(1.),
                },

                imu_gravity_time_constant = 10.,

                submaps = {
                    num_range_data = 90,
                    grid_options_2d = {
                    grid_type = "PROBABILITY_GRID",
                    resolution = 0.05,
                    },
                    range_data_inserter = {
                    range_data_inserter_type = "PROBABILITY_GRID_INSERTER_2D",
                    probability_grid_range_data_inserter = {
                        insert_free_space = true,
                        hit_probability = 0.55,
                        miss_probability = 0.49,
                    },
                    },
                },
            }
        trajectory_builder_3d = TRAJECTORY_BUILDER_3D,
        pure_localization = false,
    }
    

    TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 10

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
```
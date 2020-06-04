# Leonard Chen 的注释

<table><td bgcolor=yellow><font color=black>一点思考：Cartographer在检测回环的时候，其实就是只要一帧数据与submap匹配上了就是回环，个人感觉这里可以加一个连续性检测，就像ORB-SLAM一样，连续几帧与同一个submap匹配上了，我才算一个回环，会不会更鲁棒一些呢？</font></td></tr></table>

以下是学习过程中随手写的笔记，比较乱。

- [Leonard Chen 的注释](#leonard-chen-的注释)
  - [Local 2D Slam](#local-2d-slam)
  - [Pose Extrapolator](#pose-extrapolator)
  - [Global 2D Slam](#global-2d-slam)
  - [Local 3D Slam](#local-3d-slam)
  - [Global 3D Slam](#global-3d-slam)
  - [地标与GPS](#地标与gps)
  - [多轨迹slam](#多轨迹slam)
  - [Cartographer_ros](#cartographer_ros)
  - [配置文件](#配置文件)
    - [backup_2d.lua](#backup_2dlua)
    - [map_bulider.lua](#map_buliderlua)
    - [pose_graph.lua](#pose_graphlua)
    - [trajectory_bulider.lua](#trajectory_buliderlua)
    - [trajectory_builder_2d.lua](#trajectory_builder_2dlua)
    - [trajectory_builder_3d.lua](#trajectory_builder_3dlua)
    - [backup_3d.lua](#backup_3dlua)

## Local 2D Slam
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


## Pose Extrapolator

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

## Global 2D Slam
```
主要是构建位姿图，并优化： Sparse pose adjustment （SPA）

在localslam中 往submap里插点云的node（位姿），会插入到 pose-graph 中

插入node时执行：
  trajectory-nodes-加入新node、往submap-data-加入新submap（如果有的话）
  并给 constraint bulider 发布任务：给node和submap计算约束


计算约束：
  给优化问题新增submap，主要设置新增submap的全局位姿的初始值，会优化的
  用优化问题中的解出的最新的submap的全局位姿来推理：Tgw_submap2 = Tgw_submap1 * Tlw_submap1' * Tlw_submap2

  给优化问题加入这个新node，并设置属性：
    1.node在local-frame中2d位姿：不变的 Tlw_node_2d
    2.node在global-frame中2d位姿：设置一个初值，会优化的。
      初始值用该node所在submap在优化问题中优化值来推理：Tgw_node_2d = Tgw_submap1 * Tlw_submap1' * Tlw_node_2d
    3.node在imu-frame中的旋转：不变的 Rimu_node

    注意：Tlw_node_2d= F3d->2d(Tlw_node*Rimu_node'),这就是localslam中ceres优化的结果（注意此时是少了一个IMU中的yaw角）
  
  
  设置 node 与 submap（localslam输入的两个或一个） 的联系
  优化问题新增约束：即优化中边的观测值 Tlw-submap'*Tlw-node-2d
    * spa公式： Tij' * Ti' *Tj （cartographer用的不是这个公式，详见https://google-cartographer.readthedocs.io/en/latest/cost_functions.html）
    * 在cartographer中，Tsubmap-node_2d就是 Tij，就是边的观测值
    * Ti就是 Tgw-submap；Tj就是 Tgw-node    

  分枝定界找约束：
    从所有完成的submap（包括不同轨迹的）中，找新node与submap约束，另外这些已完成的submap一定没有当前的node
    找一下新完成的submap与之前的node（包括不同轨迹的，但不包括源生于此submap的node）的约束

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

## Local 3D Slam

这篇博客写得可以 [cartographer 3D scan matching 理解](https://www.cnblogs.com/mafuqiang/p/10885616.html)

```
local-3d-slam 是 local-2d-slam 的延伸，写下不同的地方

* correlative-scan-match:
  2d中为了加速，先对点云做二维旋转，得到 r 组点云，再分别平移，得到 r*x*y组 点云，然后分别算得分
    先旋转的原因在于只需要调用r×n次 sin、cos函数（n为点云个数）；若先平移再旋转则要调用 x*y*r*n次 sin、cos函数
  3d中就纯粹暴力地算出 x*y*z*rx*ry*rz 种刚体变换矩阵，然后分别乘到点云上算得分，（注意旋转 用的是 旋转向量 ，而不是sin、cos）
    代码实现中，估计作者觉得也没用到三角函数，纯粹都是线性代数，就没必要向2d那样调整顺序了。

* submap3d的实现
  3d由两张不同分辨率的hybridMap组成：高分辨率地图（如0.1m），低分辨率地图（如0.45m）
    其中，高分辨率地图不会管距离比较远的点（如20m外的点）
    hybridMap实现了3d栅格地图，为了节约内存，只会对用到的cell申请内存（类似八叉树），详见 bybrid_grid.h
  2d中新建submap在localframe中的位姿为 (新建submap时机器人在localframe中的 x，y; 旋转角度设置为0)，2维地图的平面只要与水平面平行就是了
  3d中新建submap的位姿：平移量依然是 (机器人在localframe中的 x，y，z)，而旋转是IMU测出的旋转（机器人在IMU坐标系下的旋转）

* ceres-scan-match
  与2d思想一致，基于非线性优化的扫描匹配，因此要对栅格地图做插值才能求导
  作者提到，由于ceres-scan-match的性能原因，才给submap3d加了高低分辨率的设定
  且在默认配置文件中，costFunction中低分辨率地图的权值比高分辨率地图的权值高
  我的理解是：3d中，优化非常消耗时间，低分辨率地图 较高的权值能让优化算法较快地迭代出 一个较好的位姿，然后高分辨率地图再去进一步优化
```

## Global 3D Slam

```
与2d一样，建立pose-graph
使用分枝定界scan-match寻找node与submap的回环约束，问题在于无法像2d那样（对点云先做预旋转，再多尺度二维平移），
3d中有三个旋转自由度，暴力遍历太浪费时间了
所以3d中通过 Rotational-scan-match 实现点云与submap的yaw对齐（3d中的imu非常重要，有了它的观测，我们才能这么大胆地认为重力方向是基本对齐的），然后再多尺度三维平移

Rotational-scan-match 会对 每一帧点云 通过z方向 进行切片，分成多层点云，并对每层点云统计直方图，这些直方图叠加起来得到一帧点云的直方图（具体看rotational_scan_match.cc）
直方图的获取方法：
  对点云中的点遍历：
    当前点云坐标c，参考点云坐标b，点云质心a
    直方图横轴：c相对a的角度（会将180度分成120份），直方图纵轴：cb与ca的角度（越接近直角，打分越高）
我们会发现这种直方图对平移不敏感，也就是说 理想情况下 一台车在同一场景下搜集点云，不过车怎么平移旋转，获得的直方图只是相差一个角度罢了
不知道是谁想的，是那篇论文提出的想法？

submap的直方图就是其里面的node的直方图旋转至global-frame中累加得到的
这样以来，一帧点云经预yaw旋转后 是不是与submap足够类似，可以通过计算 node旋转后的直方图 与 submap直方图 的余弦相似度来判断。
得分足够高的 经预yaw旋转的点云 才能进行 多尺度的三维平移匹配

（下面的内容还不太确定）
pose-graph-3d 中还有一处与 2d 不同，用户可以配置是否优化 IMU的固有误差、重力常数
若优化，则会加入以下两种边 
  角速度积分观测的相邻近的两个node旋转变换 
  角速度与加速度计观测的相邻近的三个node的速度差（v23-v12）
若不优化，则会加入以下两种边 
  localslam中获取的相邻近的两个node位姿变换
  odom中获取的相邻近的两个node位姿变换
```

## 地标与GPS
```
cartographer中还考虑了，地标、GPS的信息
可以看出来，地标的观测应该没有node那么频繁：所以我们可以通过对 观测到地标时刻 的前后两个node 拿来线性插值得到 观测到地标时刻的机器人位姿
以 观测量为边 ，链接三个顶点：两个node、一个地标
另外，要注意，地标的信息默认已经处理好了数据关联问题

其实说是gps不太准确，应该是任意能提供机器人位姿的传感器，如室内的uwb估计也行,以下就用fix作为对这类传感器的总称
使用fix要注意两个问题：
* fix的数据更新会比较频繁，所以对于待优化的node，根据其时间戳，对fix的线性插值，来获得那个时刻的 Tfix_node
* fix_frame 相对 global_frame 变换（也就是fix_frame原点在global_frame中的位姿）是不确定的，所以要优化（旋转只优化yaw轴），
  其初值根据第一个 Tfix_node*Tglobal_node' （旋转只管yaw轴）来确定，或者从文件读取
也就是说 Tfix_node 是测量值，链接两个顶点：Tglobal_node 与 Tfix_global
看起来是算法比较相信 fix 给的数据的（废话）
另外，考虑到gps 没有给旋转数据，是不是旋转的权重要设置成0？
```

## 多轨迹slam

这部分有空再看吧～～

## Cartographer_ros
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

配置文件太多了，跳来跳去的奇烦，放到这里方便对照

### backup_2d.lua
```
include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
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
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 10
```

### map_bulider.lua
```
include "pose_graph.lua"

MAP_BUILDER = {
  use_trajectory_builder_2d = false,
  use_trajectory_builder_3d = false,
  num_background_threads = 4,
  pose_graph = POSE_GRAPH,
}
```

### pose_graph.lua
```
POSE_GRAPH = {
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
}
```

### trajectory_bulider.lua
```
include "trajectory_builder_2d.lua"
include "trajectory_builder_3d.lua"

TRAJECTORY_BUILDER = {
  trajectory_builder_2d = TRAJECTORY_BUILDER_2D,
  trajectory_builder_3d = TRAJECTORY_BUILDER_3D,
  pure_localization = false,
}
```

### trajectory_builder_2d.lua

```
TRAJECTORY_BUILDER_2D = {
  use_imu_data = true,
  min_range = 0.,
  max_range = 30.,
  min_z = -0.8,
  max_z = 2.,
  missing_data_ray_length = 5.,
  num_accumulated_range_data = 1,
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
```

### trajectory_builder_3d.lua
```
MAX_3D_RANGE = 60.

TRAJECTORY_BUILDER_3D = {
  min_range = 1.,
  max_range = MAX_3D_RANGE,
  num_accumulated_range_data = 1,
  voxel_filter_size = 0.15,

  high_resolution_adaptive_voxel_filter = {
    max_length = 2.,
    min_num_points = 150,
    max_range = 15.,
  },

  low_resolution_adaptive_voxel_filter = {
    max_length = 4.,
    min_num_points = 200,
    max_range = MAX_3D_RANGE,
  },

  use_online_correlative_scan_matching = false,
  real_time_correlative_scan_matcher = {
    linear_search_window = 0.15,
    angular_search_window = math.rad(1.),
    translation_delta_cost_weight = 1e-1,
    rotation_delta_cost_weight = 1e-1,
  },

  ceres_scan_matcher = {
    occupied_space_weight_0 = 1.,
    occupied_space_weight_1 = 6.,
    translation_weight = 5.,
    rotation_weight = 4e2,
    only_optimize_yaw = false,
    ceres_solver_options = {
      use_nonmonotonic_steps = false,
      max_num_iterations = 12,
      num_threads = 1,
    },
  },

  motion_filter = {
    max_time_seconds = 0.5,
    max_distance_meters = 0.1,
    max_angle_radians = 0.004,
  },

  imu_gravity_time_constant = 10.,
  rotational_histogram_size = 120,

  submaps = {
    high_resolution = 0.10,
    high_resolution_max_range = 20.,
    low_resolution = 0.45,
    num_range_data = 160,
    range_data_inserter = {
      hit_probability = 0.55,
      miss_probability = 0.49,
      num_free_space_voxels = 2,
    },
  },
}
```

### backup_3d.lua
```
options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = true,
  publish_frame_projected_to_2d = false,
  use_odometry = false,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 0,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 2,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

TRAJECTORY_BUILDER_3D.num_accumulated_range_data = 160

MAP_BUILDER.use_trajectory_builder_3d = true
MAP_BUILDER.num_background_threads = 7
POSE_GRAPH.optimization_problem.huber_scale = 5e2
POSE_GRAPH.optimize_every_n_nodes = 320
POSE_GRAPH.constraint_builder.sampling_ratio = 0.03
POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 10
POSE_GRAPH.constraint_builder.min_score = 0.62
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.66
```
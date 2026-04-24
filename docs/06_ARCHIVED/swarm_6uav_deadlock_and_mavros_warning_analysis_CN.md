# 6机周旋锁死与 Estimator source 8 告警分析

## 背景

在 6 架无人机扩展场景中，当前已经观察到两个现象：

1. 无人机之间的最近间距已经被拉开，但交汇阶段会出现长时间周旋，表现为互相“锁死”，整体抵达终点时间显著增加。
2. `ERROR [mavlink] Estimator source 8 not supported. Unable to publish pose and velocity` 仍然出现。

这两个问题表面上看是独立的，但实际上都和“链路耦合太强、系统没有足够的破局机制”有关：

- 告警来自 MAVROS / PX4 的视觉里程计或里程计回灌路径。
- 周旋锁死来自群体优化中的对称代价和缺少调度优先级。

下面分开分析。

## 一、Estimator source 8 告警为什么还会出现

### 1.1 PX4 对 estimator source 8 的含义

PX4 的 ODOMETRY 处理逻辑里，`estimator_type = 8` 对应 `MAV_ESTIMATOR_TYPE_AUTOPILOT`。PX4 在接收外部 ODOMETRY 时，只接受少数几类来源，例如 VISION / VIO / MOCAP / UNKNOWN；如果收到 AUTOPILOT 作为外部来源，就会拒绝并报出类似警告。

换句话说，这个告警不是“6 机多了所以坏了”，而是“某条 MAVLink 里程计消息的来源类型不对”。

### 1.2 告警的真实触发链路

当前 workspace 中已经做过一层处理：在 swarm 场景里把 MAVROS 的 `odom` 插件黑名单化，避免某些 loopback 路径把 PX4 自己的 ODOMETRY 再送回 PX4。

但是这只能堵住一条入口，不能自动消灭所有可能的回灌路径。workspace 里仍然存在其他会接入 MAVROS 的桥接节点，尤其是 VINS 相关链路：

- `src/clean_uav_core/scripts/multi_vins_bridge.py`
- `src/clean_uav_core/launch/phase3_vins_pipeline.launch`

`multi_vins_bridge.py` 会把上游里程计转成 `/{vehicle}_{id}/mavros/vision_pose/pose`，并且默认允许 `enable_mavros_vision_pose = true`。如果这条链路在某些 launch 组合中仍被激活，它就会让 MAVROS 继续对 FCU 发布视觉位姿 / 里程计相关消息。

因此，告警仍然存在，通常说明以下几种情况之一：

- 运行的不是纯 truth-odom 6UAV 链路，而是混入了 VINS / bridge 相关 launch。
- 有旧 launch 或旧节点没有停干净，MAVROS 仍然保留了另一路输入。
- 当前 swarm 场景里虽然黑名单了 `odom`，但其他视觉输入仍然在把 PX4 的 ODOMETRY 语义回灌成外部估计源。

### 1.3 为什么 6 机更容易暴露这个问题

6 机场景会放大两个效应：

- 节点数量增加，launch 组合更复杂，旧桥接节点残留的概率更高。
- 系统负载更高，时间戳抖动更明显，视觉 / 里程计链路更容易出现“消息来源混杂”的现象。

所以虽然从根因上看这不是“6 机专属 bug”，但 6 机会显著放大它的可见性。

## 二、6机为什么会长时间周旋、互相锁死

### 2.1 本质是对称局部最优

当前 swarm 优化的核心问题不是“没加足够大的权重”，而是“代价函数过于对称”。

V2 的 swarm 代价在 `swarmGradCostP()` 中对每一架其他无人机逐一计算椭球距离，并在距离进入 clearance 区域时才施加梯度。这个形式有几个特点：

- 它是纯 pairwise 的。
- 它是同一时刻的空间惩罚。
- 它没有显式区分“谁该让路”。
- 它没有显式惩罚“拖时间绕圈”。

结果就是，当两组无人机在交叉区域接近时，双方都认为“让一步”更优，于是优化会自然收敛到镜像绕行、同步减速、甚至互相等待的状态。

这不是数值偶然，而是目标函数本身的对称性导致的。

### 2.2 V1 和 V2 的退化方式不同

#### V1：`lambda_collision` 同时压了障碍和 swarm

V1 的 `lambda_collision` 并不是单独的群体避障权重，它在联合代价里同时放大了障碍项和 swarm 项。这会带来两个副作用：

- 你想让无人机更强地互相避让时，也一并让它们更怕障碍。
- 当交汇区域没有明显的几何捷径时，优化器会更倾向于保守解，而不是快速通过。

再叠加重启策略里 `new_lambda2_ *= 2` 的逻辑，越失败越抬高权重，就容易把系统推入“更安全，但更慢”的保守吸引域。

#### V2：`weight_swarm` 是主因，但不是唯一因子

V2 里 `weight_swarm` 是独立权重，表面上更合理，但它仍然只是“空间项”。

V2 同时还有：

- `weight_time`
- `weight_sqrvariance`
- `weight_feasibility`

这些项确实能帮助路径更平滑、时间更合理，但它们仍然不能自动打破“双方都等对方先让”的镜像对称。

所以在 6 机交叉时，单纯继续加大 `weight_swarm` 往往会发生：

- 最近间距变大了。
- 但交汇区停留时间也变长了。
- 轨迹开始在冲突边界附近反复试探。

这就是“更安全但更慢”的典型退化。

### 2.3 为什么周旋时间会在 6 机时明显增长

当无人机数量从 2 / 3 增加到 6 时，冲突图会从稀疏变稠密：

- 一个机体不再只面对一两个局部冲突源，而是面对多个同时存在的交叉约束。
- 某条轨迹避开 A 之后，很可能又进入 B 的冲突域。
- 所有机体都在同一个时间窗里抢同一个交汇空间，导致局部最优数量上升。

此时如果没有“优先级”和“错峰”机制，优化器就会不断寻找一个折中解，而折中解往往不是快速通过，而是慢速周旋。

### 2.4 现有过滤项不足以打破死锁

`swarm_filter_far_trajectories=false` 和 `swarm_acceptance_radius=-1.0` 的默认组合意味着：

- 基本上所有广播轨迹都会进入交互集合。
- 交叉场景中冲突图接近全连接。

这会增加局部最优数量，也会让优化器更容易受到远处“并不真正会撞”的轨迹干扰。

因此，当前问题不是“把所有参数都调大”就能解决，而是需要改变优化结构。

## 三、当前可用的优化项，为什么还不够

V2 里已经存在一些有价值的项：

- `weight_swarm`
- `weight_time`
- `weight_sqrvariance`
- `weight_feasibility`

它们分别对应：

- 机间避碰
- 总时长
- 控制点间距方差 / 路径形状一致性
- 可行性约束

但这些项仍然缺少三类能力：

1. 打破对称。
2. 惩罚长时间占用冲突区域。
3. 显式表达“谁让谁先过”。

所以它们适合做基础正则和安全边界，不适合单独承担 6 机交汇调度。

## 四、修复方向分析

### 4.1 告警侧的修复方向

告警侧应该优先排查并关闭所有会把 ODOMETRY / vision pose 回灌到 PX4 的路径，尤其是 VINS / bridge 相关 launch。

优先级最高的措施是：

- 继续保留 `odom` 插件黑名单。
- 对 VINS 模式下的 MAVROS 输入做模式隔离，避免 `multi_vins_bridge.py` 和 truth-odom 链路同时对同一实例生效。
- 在 6 机 swarm 场景里尽量只保留一种 odom 来源，不要同时叠加 truth-odom、VINS pose、local_position/odom。

### 4.2 周旋锁死的修复方向

修复目标不是单纯“更大避障力”，而是“更稳定地打破局部最优”。建议把优化分成四层：

1. 空间避碰层：保留 swarm clearance 和 swarm cost。
2. 时间调度层：引入让路和错峰机制。
3. 形状正则层：控制绕圈、长蛇形和不必要横摆。
4. 破局层：在连续失败时人为打散对称状态。

## 五、建议的优化项设计

下面是更适合 6 机的设计项，按优先级从高到低排列。

### 5.1 优先级 A：加入“让路优先级”项

这是最关键的破局项。

设计思路：给每架无人机一个可比较的优先级 `p_i`，并让低优先级机体在冲突区域承担更高的避让代价。优先级可以来自：

- drone_id
- 任务剩余距离
- 距离终点的紧迫度
- 上一次失败次数

一个可行的形式是：

$$
J_{yield} = \sum_{i,j} \pi_{ij} \cdot \Phi_{ij}
$$

其中：

- `\Phi_{ij}` 是 i 对 j 的冲突代价。
- `\pi_{ij}` 是方向性的权重，表示 i 是否应该优先让路给 j。

这样做的作用是把“对称博弈”变成“有方向的调度问题”。这是打破锁死最有效的方法。

### 5.2 优先级 A：加入时间错峰项

当前 swarm cost 主要惩罚同一时刻的空间接近，但不会自动鼓励错峰通过。

建议增加时间差惩罚：

$$
J_{time\_sep} = \sum_{i,j} w_{ij} \cdot \exp\left(-\frac{|t_i - t_j|}{\tau}\right) \cdot \Phi_{ij}
$$

直觉上就是：如果两架飞机在冲突区域的到达时间非常接近，就把代价抬高；如果它们自然错峰，则代价降低。

这样优化器就不会只想着“空间上分开一点”，而会开始主动选择“时间上错开”。

### 5.3 优先级 A：加入进度惩罚，压制绕圈

长时间周旋的本质，是轨迹在冲突边界附近发生了横向摆动，但没有明显推进。

建议引入进度项，例如：

$$
J_{progress} = \lambda_{prog} \cdot \max(0, d_{goal}^{pred} - d_{goal}^{now})
$$

或者更直接地惩罚横向偏差和曲率：

$$
J_{shape} = \lambda_{var} \cdot \mathrm{Var}(\Delta p) + \lambda_{curv} \cdot \int \kappa(t)^2 dt
$$

这类项的意义是：

- 让“原地转圈”变贵。
- 让“朝目标有效前进”变便宜。

V2 里已有 `weight_sqrvariance` 和 `weight_time`，它们可以作为这类思想的基础，但建议再明确加入“目标方向上的进度”或“曲率惩罚”。

### 5.4 优先级 B：引入动态邻居集，而不是全连接 swarm

当 6 架无人机全部互相建模时，冲突图过密，局部最优会迅速变多。

建议改成只考虑“未来冲突窗口内最危险的 k 个邻居”或“半径内邻居”：

- 先按未来时间窗筛选。
- 再按预测最小距离排序。
- 只保留前 k 个真正会冲突的对象。

这样可以显著减少优化问题的耦合维度，也能减少由远场无关轨迹带来的抖动。

### 5.5 优先级 B：把 V1 的 collision 拆成 obstacle 和 swarm 两个参数

V1 目前 `lambda_collision` 同时拉高障碍和 swarm，不利于单独调 swarm 行为。

建议拆成：

- `lambda_obstacle`
- `lambda_swarm`

这样可以单独提高群体避让，而不把地形 / 障碍风险一并抬高到过保守。

### 5.6 优先级 B：设置死锁打破器

当连续重规划失败达到阈值时，系统应该强制进入破局模式，而不是无限放大权重。

可选动作：

- 让某一两架机体短暂停留。
- 给低优先级机体插入临时侧向 waypoint。
- 对冲突机体施加短时速度偏置。

这个策略的目标不是“最优”，而是“恢复可行性并打破同步绕行”。

## 六、当前配置层面的优先级建议

如果先不改算法，只从参数层着手，建议优先顺序如下：

### 优先级 1

- 继续保留 `odom` blacklist。
- 确认 6 机运行时没有同时启用 VINS bridge 和 truth-odom 回灌。
- 先把告警问题从 launch 链路上彻底隔离出来。

### 优先级 2

- 保持 6 机专用的更大 swarm clearance。
- 适度再提高 `weight_swarm`，但不要无限加大。
- 同时提高 `weight_time`，避免“慢慢绕”比“直接过”更划算。

### 优先级 3

- 增大 `weight_sqrvariance` 或加入曲率约束，抑制蛇形和原地周旋。
- 打开更严格的邻居筛选策略，减少无关交互。

### 优先级 4

- 设计真正的让路 / 错峰机制。
- 在连续失败时触发破局策略。

## 七、对现有代码的落地点

如果后续要直接改代码，建议落点如下：

- 告警链路：
  - `src/clean_uav_core/scripts/multi_vins_bridge.py`
  - `src/clean_uav_core/launch/phase3_vins_pipeline.launch`
  - `src/clean_uav_core/launch/swarm_uav_sim_instance.launch`

- V1 优化器：
  - `src/bspline_opt/src/bspline_optimizer.cpp`
  - `src/clean_uav_core/launch/swarm_uav_runtime_instance.launch`

- V2 优化器：
  - `src/traj_opt_v2/src/poly_traj_optimizer.cpp`
  - `src/clean_uav_core/launch/swarm_uav_runtime_instance_v2.launch`
  - `src/clean_uav_core/config/swarm_planner_v2.yaml`

- 群体调度和死锁打破：
  - `src/ego_planner/src/ego_replan_fsm.cpp`
  - `src/plan_manage_v2/src/ego_replan_fsm.cpp`

## 八、简短结论

当前 6 机的慢速周旋不是单一参数不够大，而是“纯对称 swarm 代价 + 缺少时间/优先级/破局机制”的组合退化。

最值得优先做的是：

1. 彻底理清并隔离所有 MAVROS 视觉 / 里程计回灌链路，消灭 estimator source 8 的旁路输入。
2. 在 V2 中加入让路优先级和时间错峰项，打破镜像对称局面。
3. 把 V1 的 collision 参数拆开，避免一刀切地把障碍和群体避让一起抬高。
4. 加死锁打破器，避免连续重规划把系统拖进“安全但不前进”的吸引子。


## 九、本次已经实施的最小修复

这次先落了两个低风险改动，作为后续更完整调度机制的前置：

- V2 新增 `optimization/swarm_symmetry_gain`，并在 6UAV 场景下由生成器默认给一个很小的正偏置，用 `drone_id` 拉开权重，减少完全对称的局部最优。
- `phase3_vins_pipeline.launch` 暴露了 `enable_mavros_vision_pose` 和 `enable_fallback_local_odom`，并在 `vehicle_num >= 6` 时默认关闭，避免视觉 / 里程计桥接把 PX4 自己的 ODOMETRY 语义继续回灌回 MAVROS。

这两个修复都不是最终答案，但它们能先把“系统过于对称”和“桥接默认值过宽”这两个诱因压下去，为后续加入让路优先级、时间错峰和死锁打破器留出更稳定的基础。

# phase1_minimal_demo.launch 详解

本文档分两部分：

1. 主要结构概览——了解参数、节点与条件启动逻辑
2. 涉及文件深入解读——每个 `include` 或 `node` 对应的脚本/launch 的功能和在整体链路中的作用

---

## 一、整体结构分析

`phase1_minimal_demo.launch` 是 Clean‑room 项目中最基础的统一启动模板，被 Phase 1 单机演示以及 Phase 2 多机入
口反复复用。它现在已经高度参数化，以便不同场景下只需传递参数即可复用。

### 1. 参数区块

文件开头连续定义了 70+ 个 `<arg>`，可按照语义分组：

- **身份与命名空间**：
  - `drone_id` / `model_name`
  - `truth_odom_topic`, `truth_pose_topic`, `child_frame`
- **目标/航点参数**：
  - `target_x/y/z`, `flight_type`, `waypoint_num`, `waypoint{1,2}_{x,y,z}`
- **地图/传感器参数**：
  - `map_size_{x,y,z}`, `cloud_topic`, `camera_pose_topic`, `depth_topic`
- **是否启动模块**：
  - `use_truth_odom`, `use_planner`, `use_px4ctrl`, `use_goal_publisher`, `use_mission_monitor`
- **目标发布**：
  - `goal_topic/frame/delay/repeat/rate`
- **规划器相关**：
  - `planner_goal_topic`, `planning_bspline_topic`, `planner_realworld_experiment` 等
- **控制器话题/服务重映射**：
  - `mavros_*` 系列，`position_cmd_topic`
- **任务监控**（mission_monitor）
- **触发器/起飞**：
  - `use_direct_traj_trigger`, `traj_trigger_topic/delay/repeat/rate`
  - `use_takeoff_trigger`, `takeoff_delay`, `takeoff_topic`

> 🚩 这里的设计宗旨是**所有话题名与行为都可通过参数修改**。多机场景只需
> 用不同参数调用同一个 launch 即可。

### 2. 里程计适配器 `include`

```xml
<include if="$(arg use_truth_odom)" file="$(find clean_uav_core)/launch/phase1_truth_odom.launch">
  …
</include>
```

条件启动 `truth_odom_adapter`。它将 `/gazebo/model_states` 转为
`/truth_odom` + `/truth_pose`；`model_name`、话题名可通过参数传入。

### 3. 规划链路

- **ego_planner_node**（条件 `use_planner`）
  - 大量 `remap` 将内部节点名映射到参数指定的话题
  - 参数设置块分为 FSM、grid_map、manager、优化等多个子模块
- **traj_server**（条件 `use_planner`）
  - 只负责把规划 B‑spline 转换为 `PositionCommand` 并发布到
    `position_cmd_topic`
- **goal_point_publisher**（条件 `use_goal_publisher`）
  - 简单脚本，延迟/重复发送目标点
- **mission_progress_monitor**（条件 `use_mission_monitor`）
  - 根据配置监控无人机是否达到航点
- **traj_start_trigger**（条件 `use_direct_traj_trigger`）
  - 延迟后把当前里程计位姿发布给 planner，用于 real‑world 模式启动

### 4. 控制链路

- **px4ctrl_node**（条件 `use_px4ctrl`）
  - 通过 `remap` 将 MAVROS 话题/服务全部参数化
  - 订阅 `/traj_start_trigger` 并且加载两个参数文件：
    + `px4ctrl/config/ctrl_param_fpv.yaml`（默认飞控控制参数）
    + `clean_uav_core/config/phase1_px4ctrl_no_rc.yaml`（覆盖 `no_RC=true`
      的起飞策略）
- **phase1_takeoff_trigger**（条件 `use_takeoff_trigger`）
  - 发布 `TakeoffLand` 消息触发 px4ctrl 进入 AUTO_TAKEOFF

---

## 二、文件及脚本深入

下面按上文出现顺序，对每个外部引用展开说明。

### 2.1 phase1_truth_odom.launch

包含一个 `truth_odom_adapter.py` 节点，作用如前所述。
参数：

- `model_name`：Gazebo 模型（如 iris、iris_0）
- `odom_topic`/`pose_topic`：输出话题名
- `world_frame`/`child_frame`：里程计坐标系配置

这是唯一一个用到 Gazebo 原始接口的地方，因此任何需要真值位姿的组件都
通过该适配器获取。

### 2.2 ego_planner_node（位于 ego_planner 包）

主规划节点。几条关键逻辑线索：

- 输入：`~odom_world`、`/move_base_simple/goal`、`/traj_start_trigger`、
  `~grid_map/cloud` 等
- 输出：`~planning/bspline`、`~planning/data_display`
- FSM 参数（`fsm/…`）控制算法流程。
- 内部依赖 bspline_opt、path_searching、plan_env；但这些在 launch 文件
  中不显式出现，已通过包声明隐式加载。

### 2.3 traj_server（位于 traj_utils 包）

非常轻量，仅包含一个回调：

```cpp
void callback(const Bspline &bs) {
  PositionCommand cmd = convert(bs);
  pub.publish(cmd);
}
```

参数 `traj_server/time_forward` 决定前馈时延（1s）。

### 2.4 goal_point_publisher.py

前面阅读过，位于 `clean_uav_core/scripts`。本 launch 只是传递参数。它
在无需 RViz 时自动发送固定目标，常用于无人值守的初次测试。

### 2.5 mission_progress_monitor.py

同样位于 `clean_uav_core/scripts`。监控逻辑参考之前文档：
- 根据里程计/命令判断航点到达
- 可配置两种后备策略：命令到达或规划活跃度
其启动条件为 `use_mission_monitor`，用于需要可视化任务状态的场景。

### 2.6 traj_start_trigger.py

待 `use_direct_traj_trigger` 为真时启动；原设计用于实地飞行，
确保规划在无人机起飞稳定之后才开始。

### 2.7 px4ctrl_node

我们前面已经细读其源码；launch 文件还把几个 MAVROS service 也参数化
了。此处重点强调：

- 它通过 `remap` 把内部 `~odom`、`~cmd`、`~takeoff_land` 话题映射到
  各种命名空间
- `rosparam load` 两个 yaml，覆盖控制器参数：默认参数+no‑RC 设置

### 2.8 takeoff_land_trigger.py

位于 `clean_uav_core/scripts`。当 `use_takeoff_trigger` 为真时延迟发布起飞
命令，并重复发送以防丢包。参数包括发布频率、延迟时间和重复次数。
在仿真环境无需人为操作时非常方便。

---

## 三、使用示例

1. **单机全栈**（默认）

```bash
roslaunch clean_uav_core phase1_minimal_demo.launch
```

会自动启动适配器、规划器、控制器和触发器。目标点 (5,0,1)，8s 后起飞。

2. **仅起飞并保持**

```bash
roslaunch clean_uav_core phase1_minimal_demo.launch use_planner:=false
```

3. **RViz 手动目标**

```bash
roslaunch clean_uav_core phase1_minimal_demo.launch use_goal_publisher:=false
``` 
然后打开 RViz 并点击“2D Nav Goal”。

4. **只看规划不飞行**

```bash
roslaunch clean_uav_core phase1_minimal_demo.launch use_px4ctrl:=false
```

5. **Phase 2 通过 namespace 嵌套**

`phase2_dual_uav_stack.launch`（详见后续文档）会在两个 `group` 中引用本
文件，并将所有参数重写为 `uav0/…` 或 `/iris_0/...`。

---

## 四、心得与注意事项

- **参数顺序**：`<arg>` 的定义顺序并不影响重映射，但保持语义分组可以提高
  可读性。
- **条件包含**：`if="$(arg use_…)"` 是节省资源的关键，用于测试不同组合。
- **多机环境**：本文件是 Phase 2 的基础，所有重映射与命名空间定制都依赖于
  这里的参数逻辑。

---

下一个文档将聚焦 Phase 2 双机栈本身。
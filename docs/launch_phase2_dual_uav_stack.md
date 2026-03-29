# phase2_dual_uav_stack.launch 详解

此 launch 文件是 Phase 2 多机算法/控制栈入口，它不直接启动功能节点，
而是通过两个命名空间 (`uav0`/`uav1`) 各自 `include` 了
`phase1_minimal_demo.launch` 并对参数进行重写。此外，文件还在顶层
添加了一个用于“双机同步触发”的脚本节点以及可选的 RViz 可视化。

文档结构：

1. 主体结构与参数说明
2. `uav0`/`uav1` 组内部参数详解
3. 额外节点说明（dual_traj_start_trigger + rviz）
4. 调用示例与注意点

---

## 一、主体结构与参数说明

### 1. 全局 `<arg>` 列表

```xml
<arg name="uav0_model" default="iris_0"/>
<arg name="uav1_model" default="iris_1"/>

<arg name="takeoff_delay" default="8.0"/>
<arg name="traj_trigger_delay" default="20.0"/>
<arg name="traj_trigger_repeat" default="1"/>
<arg name="traj_trigger_rate" default="3.0"/>
<arg name="planner_max_vel" default="0.6"/>
<arg name="planner_max_acc" default="3.0"/>
<arg name="planner_max_jerk" default="4.0"/>
<arg name="bspline_limit_vel" default="2.0"/>
<arg name="bspline_limit_acc" default="3.0"/>
<arg name="bspline_limit_ratio" default="1.1"/>
<arg name="use_rviz" default="true"/>
<arg name="rviz_config" default="$(dirname)/../rviz/phase2_ego_obstacle_debug.rviz"/>

<arg name="uav0_target_x" default="-2.0"/>
<arg name="uav0_target_y" default="3.5"/>
<arg name="uav0_target_z" default="1.9"/>

<arg name="uav1_target_x" default="1.0"/>
<arg name="uav1_target_y" default="6.5"/>
<arg name="uav1_target_z" default="1.9"/>
```

以上参数覆盖了两个子机的模型名称、起飞/触发延迟、规划速度约束、
B-spline 限制以及目标位置。重点在于：**所有参数均为全局定义，
两个子机共用同一套值，并在内部 group 中分别传入**。

### 2. 命名空间分组

使用两个 `<group ns="uav0">` 与 `<group ns="uav1">` 包裹两个包含块，
每个组内部调用 `phase1_minimal_demo.launch` 并提供大量 `<arg>` 值。这样
就把相同的算法栈实例化两份，且运行在各自的 ROS namespace 下。

```xml
<group ns="uav0">
  <include file="$(find clean_uav_core)/launch/phase1_minimal_demo.launch">
    <arg name="drone_id" value="0"/>
    …  <!-- 下面的参数详细见下一节 -->
  </include>
</group>
```

组的作用：
1. 隔离话题，使得 `/uav0/planning/bspline` 与 `/uav1/planning/bspline` 不冲突。
2. 保持 MAVROS 原始话题在 `/iris_0/mavros/...` 与 `/iris_1/mavros/...` 下
   以便仿真层支持双机。
3. 每个组内还能对规划参数、监控参数做微调，例如无人机 ID、目标点、
   `takeoff_delay` 等。

---

## 二、组内部参数详解

`uav0` 组与 `uav1` 组几乎完全对称，仅有数个与编号相关的差异。
下面列出关键部分：

### 1. MAVROS 相关参数

| 参数 | uav0 value | uav1 value | 说明 |
|------|------------|------------|------|
| `mavros_state_topic` | `/iris_0/mavros/state` | `/iris_1/mavros/state` | MAVROS 状态话题 |
| `mavros_attitude_cmd_topic` | `/iris_0/mavros/setpoint_raw/attitude` | `/iris_1/...` | 控制输出 |
| `mavros_set_mode_service` | `/iris_0/mavros/set_mode` | `/iris_1/...` | 服务 | 

> 这些参数必须对应各自仿真出现的 namespace，否则 px4ctrl 不能与
> 对应飞控建立连接。

### 2. 目标与监控参数

| 参数 | uav0 默认 | uav1 默认 |
|------|----------|----------|
| `target_x/y/z` | 从全局引入（`uav0_target_*`） | 引入 `uav1_target_*` |
| `waypoint1_x/y/z` | 固定值 `5,1.5,1` | uav1 为 `5,4.5,1` （避免碰撞） |
| `reach_radius`/`hold_time` | 0.8 / 0.4 | 同上 |

### 3. 规划速度约束

两个组共用全局的 `planner_max_vel/acc/jerk` 以及 bspline 限制。
若需要让两机速度不同，可自行在组内覆写这些参数。

### 4. 起飞延迟

`takeoff_delay` 也是全局参数，在组内部直接传入。这意味着两机
会**同时**收到起飞命令；若希望先后起飞可以在组中手动改写。

### 5. 触发器参数

`traj_trigger_topic` 固定为 `traj_start_trigger`（组内不重映射），
因此触发器脚本需运行在根命名空间，见下节。

---

## 三、额外节点说明

### 3.1 dual_traj_start_trigger.py

这一脚本在文件末尾启动，没有条件语句；它负责**同步**向
two UAV 发送触发位姿，保证规划器同时开始规划。

参数配置（见 launch）

```xml
<param name="uav0_odom_topic" value="/uav0/truth_odom"/>
<param name="uav1_odom_topic" value="/uav1/truth_odom"/>
<param name="uav0_trigger_topic" value="/uav0/traj_start_trigger"/>
<param name="uav1_trigger_topic" value="/uav1/traj_start_trigger"/>
<param name="delay" value="$(arg traj_trigger_delay)"/>
<param name="repeat" value="$(arg traj_trigger_repeat)"/>
```

它在启动后等待指定延迟、收集两机最新位姿，然后按指定频率向各自
`/uavN/traj_start_trigger` 发布同一时间戳的姿态消息。多机实验中该脚本
是避免时间漂移与碰撞的关键。

### 3.2 RViz 可视化

```xml
<node if="$(arg use_rviz)" pkg="rviz" type="rviz" name="phase2_ego_rviz" args="-d $(arg rviz_config)" output="screen"/>
```

默认打开自定义 rviz 配置`phase2_ego_obstacle_debug.rviz`（位于
`clean_uav_core/rviz/`），用于同时显示两机规划轨迹与地图。

---

## 四、使用示例与注意

### 1. 启动方式

```bash
roslaunch clean_uav_core phase2_dual_uav_stack.launch
```

该命令假设仿真层已预先通过
`phase2_px4_multi_sim.launch` 启动。否则 `iris_0` 和 `iris_1` 
相关话题不会出现，MAVROS 将报错。

### 2. 修改参数

- 对某一架机独立调整可在其组内修改相应 `<arg>`，例如：

```xml
<group ns="uav1">
  <include …>
    <arg name="planner_max_vel" value="1.0"/>
  </include>
</group>
```

- 若需要先后起飞，可给两个组不同的 `takeoff_delay`。

### 3. 运行顺序

1. 启动仿真：`phase2_px4_multi_sim.launch` → Gazebo 加载两架机
2. 启动算法栈：本文件
3. 等待触发器完成（默认 delay=20 s），两机进入规划状态

**注意**：倒序启动会导致 MAVROS 无法连接 PX4，因为对应 namespace
下的话题尚不存在。

### 4. 调试提示

- 如果只有一架机飞行，检查 `/uav1/mavros/state` 是否正确发布。
- 如果触发不同步，可查看 `/uav0/traj_start_trigger` 与
  `/uav1/traj_start_trigger` 是否在同一时间戳发布。

---

## 五、小结

`phase2_dual_uav_stack.launch` 基本上是两个平行的
`phase1_minimal_demo.launch` 的组合，再加上
`dual_traj_start_trigger.py`、RViz。它的设计重点在于参数化与
命名空间隔离，使得多机演示可以在不修改任何底层节点代码的情
况下进行。

文档最后的 Phase 2 发展文档中已有更多运行验证示例，可以继续
参考。

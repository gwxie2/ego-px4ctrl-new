# XTDrone Cleanroom Phase-2 架构与原理说明（汇报版）

## 1. 文档目的

本文面向“阶段化汇报”和“原理展示”，整理当前 `cleanroom_ws` Phase-2 双机链路的：

1. 系统架构与模块职责
2. 端到端控制/规划数据流
3. 状态机关键行为与启动时序
4. 已发现问题与修复机制
5. 验证判据与日志解读方法
6. **无人机飞行速度过快时的调整方法（可指定）**

---

## 2. 总体架构（分层）

当前链路按“仿真层 → 规划层 → 控制层 → 监视层”分层：

- **仿真与飞控接口层**（SITL + Gazebo + MAVROS）
  - 启动入口：`src/clean_uav_core/launch/phase2_px4_multi_sim.launch`
  - 功能：双机模型生成、MAVROS桥接、PX4参数引导（如 `COM_RCL_EXCEPT` / `COM_RC_IN_MODE`）
- **规划层**（EGO Planner）
  - 入口：`src/clean_uav_core/launch/phase1_minimal_demo.launch` 中 `ego_planner_node` + `traj_server`
  - 功能：目标点→局部轨迹（bspline）→位置命令 `position_cmd`
- **控制层**（px4ctrl）
  - 入口：`phase1_minimal_demo.launch` 中 `px4ctrl_node`
  - 功能：接收 `position_cmd`，执行姿态/推力控制，驱动 `/mavros/setpoint_raw/attitude`
- **任务与同步层**
  - 同步触发：`dual_traj_start_trigger.py`
  - 起飞触发：`takeoff_land_trigger.py`
  - 进度监视：`mission_progress_monitor.py`

核心思想：
- 起飞与规划触发解耦
- 规划与控制通过 `position_cmd` 解耦
- 多机通过命名空间隔离并统一上层触发策略

---

## 3. 端到端数据流（关键话题）

### 3.1 控制主链路

1. 目标参数（`fsm/waypoint*`）注入 `ego_planner_node`
2. `ego_planner_node` 生成 `planning/bspline`
3. `traj_server` 将 bspline 转为 `position_cmd`
4. `px4ctrl` 订阅 `~cmd`（映射到 `position_cmd`）
5. `px4ctrl` 输出姿态推力到 MAVROS setpoint
6. PX4 执行控制，反馈状态与位姿

### 3.2 当前关键重映射

- 在 `phase1_minimal_demo.launch` 中对 `traj_server` 同时映射：
  - `position_cmd -> $(arg position_cmd_topic)`
  - `/position_cmd -> $(arg position_cmd_topic)`

这一步修复了“规划在跑、无人机不动”的根因之一（绝对话题与命名空间话题不一致）。

---

## 4. 状态机与启动时序原理

### 4.1 px4ctrl 关键状态（简化）

- `MANUAL_CTRL`：初始态
- `AUTO_TAKEOFF`：自动起飞
- `AUTO_HOVER`：悬停等待命令
- `CMD_CTRL`：执行轨迹命令

当前行为改进点：
- 在 `AUTO_HOVER` 且收到命令时，若未在 `OFFBOARD`，主动尝试切换 `OFFBOARD`，再进入 `CMD_CTRL`。

### 4.2 EGO FSM 关键状态

- `INIT -> WAIT_TARGET -> GEN_NEW_TRAJ -> EXEC_TRAJ`
- 当收到 trigger 或目标更新时，进入重规划循环

### 4.3 启动策略

当前默认是“先起飞、后触发规划”：
- `takeoff_delay` 控制起飞触发时间
- `traj_trigger_delay` 控制双机同步规划触发时间
- `traj_trigger_repeat` 默认设为 1，减少重复触发导致的状态抖动

---

## 5. 近期问题与对应修复

### 问题A：起飞后不动

现象：日志出现 `EXEC_TRAJ`，但飞机基本悬停。

根因与修复：
1. `traj_server` 输出话题与 `px4ctrl` 订阅不一致（绝对/相对话题混用）
   - 修复：补充 `/position_cmd` 的显式重映射。
2. `AUTO_HOVER` 到 `CMD_CTRL` 依赖模式状态切换时机
   - 修复：在收到命令时主动尝试进入 `OFFBOARD`。

### 问题B：mission 提前完成

现象：还未真实运动，`mission_status` 已 completed。

根因与修复：
- `mission_progress_monitor` 的 planner activity 超时基准过早。
- 修复：以“首次 bspline 出现时间”作为超时计时起点，且默认关闭此 fallback。

### 问题C：IMU 频率告警持续

现象：此前阈值为 100Hz，SITL 下频繁告警。

修复：
- 调整告警阈值为 30Hz，并打印当前实测频率。
- 该告警仍可能出现（如 10~20Hz），代表真实仿真负载较高，不再是“误报阈值”问题。

> 注：你已恢复 `iris_stereo_camera` 用于可视化，这是合理需求；但它会提高仿真负载，IMU频率下降风险会更高。

---

## 6. 验证判据（建议在汇报中使用）

判定“规划与控制闭环成功”的证据建议同时满足：

1. **状态机证据**
   - `MANUAL_CTRL -> AUTO_TAKEOFF -> AUTO_HOVER -> CMD_CTRL`
   - `EGO FSM: WAIT_TARGET -> GEN_NEW_TRAJ -> EXEC_TRAJ`
2. **命令证据**
   - `/uav*/position_cmd` 有持续输出
3. **位姿证据（强）**
   - 起点与终点 `truth_odom` 比较：至少 `x/y` 有显著变化，且接近目标点
4. **任务证据**
   - `/uav*/mission_status` 进入 `mission completed`

---

## 7. 重点问答：飞行速度太快怎么调？可以指定吗？

可以，且现在已支持通过 `roslaunch` 参数指定。

### 7.1 速度由哪些参数决定

在 `phase1_minimal_demo.launch` 中，EGO相关的速度上限主要由以下参数决定：

- `manager/max_vel`
- `optimization/max_vel`
- `bspline/limit_vel`

加速度相关：
- `manager/max_acc`
- `optimization/max_acc`
- `bspline/limit_acc`

jerk相关：
- `manager/max_jerk`

这些参数共同影响轨迹的“快慢”和“激进程度”。

> 起飞阶段速度是 `auto_takeoff_land/takeoff_land_speed`，只影响起降，不是巡航轨迹速度。

### 7.2 当前已支持的可指定参数

在 `phase2_dual_uav_stack.launch` 顶层新增了：

- `planner_max_vel`（默认 2.0）
- `planner_max_acc`（默认 3.0）
- `planner_max_jerk`（默认 4.0）
- `bspline_limit_vel`（默认 2.0）
- `bspline_limit_acc`（默认 3.0）
- `bspline_limit_ratio`（默认 1.1）

并传递到每架无人机实例。

### 7.3 推荐调参方式（从稳到快）

如果你觉得“太快”，建议先把速度上限降到 `1.0~1.2`：

```bash
roslaunch clean_uav_core phase2_dual_uav_stack.launch \
  uav0_target_x:=-2.0 uav0_target_y:=3.5 uav0_target_z:=1.9 \
  uav1_target_x:=1.0 uav1_target_y:=6.5 uav1_target_z:=1.9 \
  planner_max_vel:=1.0 \
  bspline_limit_vel:=1.0 \
  planner_max_acc:=2.0 \
  bspline_limit_acc:=2.0 \
  planner_max_jerk:=3.0
```

若仍偏激进，再降低 `planner_max_acc` 和 `planner_max_jerk`。

### 7.4 参数调整原则

- 先调速度上限，再调加速度，再调jerk。
- `manager/max_*` 与 `bspline/limit_*` 保持同量级，避免相互“打架”。
- 可视化模型负载高时，低频 IMU 会影响控制效果，建议用更保守速度上限。

---

## 8. 当前阶段可汇报要点（建议）

1. 已建立双机“先起飞后规划”稳定时序。
2. 已闭合“规划命令→控制执行”链路（CMD_CTRL可达）。
3. 已修复任务监视提前完成问题。
4. 已将速度上限参数化，支持现场指定演示速度。
5. 已形成可复用的日志判据与验证流程。

---

## 9. 后续建议（暂停开发前）

1. 固化一套“演示参数预设”（高速/稳健/可视化模式）。
2. 将关键日志检查命令写入单独脚本，便于汇报前一键自检。
3. 如果长期使用 `iris_stereo_camera`，可考虑降低仿真负载或分离可视化与控制运行模式。

---

（文档版本：Phase-2 阶段汇报版）

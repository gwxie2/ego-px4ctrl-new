# Phase 2 运行日志判读与成功判据（实操版）

本文档针对 `clean_uav_core phase2_px4_multi_sim.launch` + `clean_uav_core phase2_dual_uav_stack.launch` 的双机运行，说明：

1. 终端输出如何分层阅读；
2. 你给出的典型告警/报错如何判断“正常暂态”还是“异常”；
3. 运行成功的判据与最小验收命令。

---

## 1. 推荐启动顺序（避免误判）

### 1.1 终端 A：仿真层（PX4 + Gazebo + MAVROS）

```bash
cd /home/guanwen/XTDrone/cleanroom_ws
source tools/source_phase1_env.sh
roslaunch clean_uav_core phase2_px4_multi_sim.launch gui:=true
```

无图形模式（服务器/远程）可改为：

```bash
roslaunch clean_uav_core phase2_px4_multi_sim.launch gui:=false
```

### 1.2 终端 B：算法与控制层

等待终端 A 出现 MAVROS 正常连接后再启动：

```bash
cd /home/guanwen/XTDrone/cleanroom_ws
source tools/source_phase1_env.sh
roslaunch clean_uav_core phase2_dual_uav_stack.launch
```

---

## 2. 你给出的日志片段如何解读

### 2.1 `model 'iris_0/iris_1' not found in /gazebo/model_states`

示例：

- `[WARN] ... model 'iris_0' not found in /gazebo/model_states`
- `[WARN] ... model 'iris_1' not found in /gazebo/model_states`

结论：

- **短时间出现（启动初期 1~10 秒）通常是正常暂态**，因为模型尚未完成 spawn。
- **持续出现**则异常，通常意味着：
  1. Gazebo 模型未成功加载；
  2. `sdf` 模型名不存在；
  3. 环境变量（`GAZEBO_MODEL_PATH`）未正确指向 PX4 模型目录。

### 2.2

- `[FSM]: state: INIT`
- `no odom.`
- `wait for goal or trigger.`

结论：

- 这三行在启动初期是常见状态，表示 planner 还没拿到 odom/trigger。
- 如果长时间停留（>20s）且没有进入 `EXEC_TRAJ`，就要检查 odom、trigger、MAVROS 连接。

### 2.3 `takeoff_land_trigger published takeoff (x/6)`

结论：

- 该日志本身是**正常行为**，表示 takeoff 触发器在发命令。
- 是否真正生效，要结合 MAVROS 状态（`connected/armed/mode`）确认。

### 2.4 `[ERROR] Unable to connnect to PX4!!!`

结论：

- 这是**异常**，表示 `px4ctrl` 无法通过 MAVROS 与 PX4 建立有效连接。
- 常见原因：
  1. 终端 B 启动太早（仿真层尚未 ready）；
  2. MAVROS 的 `fcu_url` 未连通；
  3. PX4 SITL 未正常启动；
  4. 端口冲突或残留进程干扰。

---

## 3. “Gazebo 没窗口”是否正确？

是可能且正确的，取决于参数：

- `phase2_px4_multi_sim.launch` 当前默认 `gui:=false`。
- 你“去掉命令行里的 gui 参数”时，会回落到默认值，因此依然不显示窗口。

若要强制显示：

```bash
roslaunch clean_uav_core phase2_px4_multi_sim.launch gui:=true
```

如果仍不显示，需检查本机图形环境（`DISPLAY`、远程会话、显卡驱动）。

---

## 4. “SDF 找不到路径”是否正确？

不是期望行为，通常是环境配置问题。

当前默认模型是：

- `sdf:=iris_stereo_camera`
- 期望目录：`<PX4_ROOT>/Tools/sitl_gazebo/models/iris_stereo_camera/iris_stereo_camera.sdf`

已在 `tools/source_phase1_env.sh` 中补充：

- `GAZEBO_MODEL_PATH`
- `GAZEBO_PLUGIN_PATH`

并支持显式指定 PX4 根目录：

```bash
export PHASE1_PX4_ROOT=/home/guanwen/PX4_Firmware
source /home/guanwen/XTDrone/cleanroom_ws/tools/source_phase1_env.sh
```

如果你维护的是另一份 PX4（例如 `XTDrone/PX4_Firmware`），可切换为对应路径再启动。

---

## 5. 要不要改去 PX4_Firmware 路径直接运行？

建议：

- **正常联调优先继续在 `cleanroom_ws` 路径运行**，因为这里已整合双机 launch、remap 和监控判据。
- 仅在以下场景切回 PX4 根目录单独验证：
  1. 需要先证明“纯 PX4 SITL + Gazebo”单独可启动；
  2. 怀疑 cleanroom launch 之外的 PX4 构建/插件损坏。

等价地，你也可以在 cleanroom 中通过 `PHASE1_PX4_ROOT` 指向你想要的 PX4 根目录，不必改工作目录。

---

## 6. 成功判据（最小可执行）

### 6.1 仿真层成功判据

至少满足：

1. Gazebo 存在（GUI 或 headless 均可）；
2. `/gazebo/model_states` 中出现 `iris_0`、`iris_1`；
3. `/iris_0/mavros/state`、`/iris_1/mavros/state` 的 `connected: True`。

命令：

```bash
source tools/source_phase1_env.sh
rostopic echo -n 1 /iris_0/mavros/state
rostopic echo -n 1 /iris_1/mavros/state
```

### 6.2 算法/控制层成功判据

至少满足：

1. 日志中出现 `dual_traj_start_trigger published synchronized trigger`；
2. FSM 至少出现过 `state: EXEC_TRAJ`；
3. `/uav0/mission_status`、`/uav1/mission_status` 最终为 `phase1_mission: mission completed`。

命令：

```bash
source tools/source_phase1_env.sh
rostopic echo -n 1 /uav0/mission_status
rostopic echo -n 1 /uav1/mission_status
```

---

## 7. 终端阅读建议（高效排障）

按优先级阅读：

1. **先看连接性**：`Unable to connnect to PX4`、MAVROS state `connected`。
2. **再看时序**：是否先有 odom/model，再有 trigger，再有 EXEC_TRAJ。
3. **最后看收敛**：是否出现 `mission completed`。

建议保留两份日志：

- `/tmp/phase2_px4_multi_sim.log`
- `/tmp/phase2_dual_uav_stack.log`

并用关键词过滤：

```bash
grep -E 'Unable to connnect to PX4|model.*not found|EXEC_TRAJ|mission completed|synchronized trigger' /tmp/phase2_dual_uav_stack.log
```

---

## 8. 一句话结论

- 你给出的片段里，`model not found` 与 `FSM INIT/no odom` 可以是暂态；
- `Unable to connnect to PX4!!!` 是硬异常，优先处理连接时序与 MAVROS/PX4 连通；
- 按本文启动顺序与判据执行，可快速判断是“正常启动过程”还是“真正失败”。

---

## 9. 当前状态机启动顺序与预期时延（重点）

你描述的“先看到 EGO 规划，再过约 10 秒起飞，随后悬停不走”与旧默认时序一致：

- 旧配置中 `planner_realworld_experiment=false`，EGO 不等待 trigger，会提前进入规划；
- 起飞触发默认 `takeoff_delay=8s`，所以容易出现“先规划、后起飞”。

已调整为“先起飞，再触发规划”：

1. `px4ctrl`：接收 `takeoff_land_trigger` 后进入自动起飞过程；
2. EGO：设置 `planner_realworld_experiment=true`，等待 `/traj_start_trigger`；
3. 双机同步触发器：默认 `traj_trigger_delay=20s` 后发布 trigger；
4. 触发后 EGO 才从 `INIT/WAIT_TARGET` 进入 `GEN_NEW_TRAJ/EXEC_TRAJ`，无人机开始向目标点运动。

### 9.1 默认时延参数（Phase 2）

- 起飞触发延迟：`takeoff_delay=8.0s`
- 轨迹触发延迟：`traj_trigger_delay=20.0s`
- 起飞高度/速度（px4ctrl）：`takeoff_height=1.0m`，`takeoff_land_speed=0.3m/s`

经验上，`20s` 触发延迟通常足够覆盖解锁 + 起飞稳定窗口；若机器较慢可增大到 `24~30s`。

---

## 10. 固定目标点如何指定

Phase 2 双机固定目标点在 launch 顶层参数中指定（推荐）：

文件：`cleanroom_ws/src/clean_uav_core/launch/phase2_dual_uav_stack.launch`

- `uav0_target_x / uav0_target_y / uav0_target_z`
- `uav1_target_x / uav1_target_y / uav1_target_z`

### 10.1 命令行覆盖示例

```bash
roslaunch clean_uav_core phase2_dual_uav_stack.launch \
  uav0_target_x:=4.0 uav0_target_y:=0.5 uav0_target_z:=1.2 \
  uav1_target_x:=4.0 uav1_target_y:=3.5 uav1_target_z:=1.2
```

### 10.2 同步调时延示例

```bash
roslaunch clean_uav_core phase2_dual_uav_stack.launch \
  takeoff_delay:=8.0 traj_trigger_delay:=24.0
```

如果你希望严格“先起飞完全稳定再动”，优先增加 `traj_trigger_delay`，而不是一味提前 `takeoff_delay`。

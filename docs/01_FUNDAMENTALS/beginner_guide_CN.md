# 新手入门教程（中文）

> 本文档面向刚接触本仓库的新开发者。阅读完毕后你应该能够：
> 1. 从零搭建运行环境
> 2. 理解系统的整体数据流
> 3. 独立运行 Phase 1 单机仿真
> 4. 进行基本的调参和出错排查

---

## 第一章：前置知识

### 1.1 你需要了解的背景概念

| 概念 | 一句话解释 |
|------|-----------|
| **ROS Noetic** | 机器人操作系统，负责节点间通信（发布/订阅话题、Service调用）|
| **Catkin** | ROS 的构建系统，`catkin_make` 编译整个工作区 |
| **PX4 SITL** | PX4 飞控固件的软件在环仿真，在电脑上模拟飞控硬件 |
| **Gazebo** | 3D 物理仿真器，模拟无人机飞行物理过程 |
| **MAVROS** | MAVLink ↔ ROS 协议转换桥，让 ROS 节点能控制 PX4 飞控 |
| **EGO-Planner** | 基于 B 样条的在线轨迹规划算法，生成平滑可执行的飞行路径 |
| **px4ctrl** | 接收轨迹指令并转换为姿态控制指令发给飞控的位置控制器 |
| **clean_uav_core** | 本项目的"胶水包"，通过 launch 文件把上述所有组件串联起来 |

### 1.2 目录结构速览

```
cleanroom_ws/         ← 你的工作区根目录
├── src/              ← 所有 ROS 包放这里
│   ├── clean_uav_core/   ← ★ 最重要，launch 文件和脚本都在这里
│   ├── quadrotor_msgs/   ← 自定义消息包（已定制，勿修改）
│   ├── px4ctrl/          ← 符号链接，指向 ~/XTDrone/px4ctrl
│   └── ego_planner/      ← 符号链接，指向 EGO 规划器
├── docs/             ← 详细文档（就是你现在看的）
├── tools/            ← 环境变量脚本
└── build/ devel/     ← 构建产物（自动生成，不要提交到 git）
```

---

## 第二章：环境搭建

### 2.1 前置依赖确认

本教程假设你已经按照 XTDrone 官方指引安装好了：
- Ubuntu 20.04
- ROS Noetic（完整桌面版 `ros-noetic-desktop-full`）
- PX4 固件（位于 `~/XTDrone/PX4_Firmware`）
- Gazebo 插件（`sitl_gazebo`）
- MAVROS（`ros-noetic-mavros` + `ros-noetic-mavros-extras`）

**验证安装：**

```bash
rosversion -d        # 应输出 noetic
gazebo --version     # 应输出 Gazebo 11.x
ls ~/XTDrone/PX4_Firmware/  # 应有 launch/ 目录
```

### 2.2 初始化工作区（首次）

```bash
cd ~/XTDrone/cleanroom_ws

# 【重要】设置环境变量（每次新终端都需要执行）
source tools/source_phase1_env.sh

# 构建整个工作区（首次约需 3~5 分钟）
catkin_make -j4

# 验证构建成功
echo "构建完成，关键可执行文件："
ls devel/lib/px4ctrl/             # 应有 px4ctrl_node
ls devel/lib/ego_planner/         # 应有 ego_planner_node, traj_server
```

### 2.3 每次新终端的必要操作

```bash
# 方式一：source 环境脚本（推荐写入 ~/.bashrc）
source ~/XTDrone/cleanroom_ws/tools/source_phase1_env.sh

# 或者写入 ~/.bashrc 永久生效：
echo "source ~/XTDrone/cleanroom_ws/tools/source_phase1_env.sh" >> ~/.bashrc
```

---

## 第三章：运行第一次仿真（Phase 1 单机）

### 3.1 理解将要发生什么

执行一次 `roslaunch` 后，系统会在后台依次启动以下节点：

```
t=0s    Gazebo 启动，加载 iris 无人机模型
        PX4 SITL 启动，连接 MAVROS
t=3s    truth_odom_adapter 开始桥接里程计
        ego_planner_node 初始化（等待里程计）
t=5s    px4_param_bootstrap 设置 PX4 参数（禁用RC丢失保护）
t=10s   takeoff_land_trigger 发送起飞指令
t=11s   px4ctrl 请求 Offboard 模式，电机怠速 2.8s
t=14s   无人机爬升至 1.0m 目标高度
t=15s   traj_start_trigger 触发规划器
t=15s   ego_planner 开始规划到目标点 (5,0,1)
        无人机开始飞行到目标
```

### 3.2 执行步骤

```bash
# 终端1：启动全栈仿真
source ~/XTDrone/cleanroom_ws/tools/source_phase1_env.sh
cd ~/XTDrone/cleanroom_ws
roslaunch clean_uav_core phase1_fullstack.launch

# 等待约 15 秒，观察终端输出：
# [clean_uav_core] truth_odom_adapter locked model 'iris' at index X
# [px4ctrl] ... AUTO_TAKEOFF ...
# [clean_uav_core] traj_start_trigger published trigger 1/5
# [px4ctrl] ... CMD_CTRL ...
```

### 3.3 监控运行状态

```bash
# 终端2：查看关键话题
source ~/XTDrone/cleanroom_ws/devel/setup.bash

# 实时查看无人机位置
rostopic echo /truth_odom/pose/pose/position

# 查看控制器状态（通过 debug 话题）
rostopic echo /px4ctrl/debug

# 查看 PX4/MAVROS 连接状态
rostopic echo /mavros/state

# 查看话题图（需要安装 rqt）
rqt_graph
```

### 3.4 修改目标点

默认目标点为 `(5, 0, 1)`，修改方式：

```bash
# 方法1：launch 参数
roslaunch clean_uav_core phase1_fullstack.launch \
  target_x:=3.0 target_y:=2.0 target_z:=1.5

# 方法2：使用 RViz Goal：先改 launch 参数 use_goal_publisher:=false，
# 然后在 RViz 中用 "2D Nav Goal" 点击目标位置
```

---

## 第四章：关键话题与消息速查

### 4.1 你最常用的话题

| 话题 | 消息类型 | 说明 |
|------|----------|------|
| `/truth_odom` | `nav_msgs/Odometry` | 无人机真值位置、速度、姿态 |
| `/mavros/state` | `mavros_msgs/State` | PX4 状态（是否 Offboard/Armed）|
| `/planning/bspline` | `traj_utils/Bspline` | 规划器输出的 B 样条轨迹 |
| `/position_cmd` | `quadrotor_msgs/PositionCommand` | traj_server 输出的位置指令 |
| `/mavros/setpoint_raw/attitude` | `mavros_msgs/AttitudeTarget` | px4ctrl 输出的姿态指令 |
| `/px4ctrl/debug` | `quadrotor_msgs/Px4ctrlDebug` | px4ctrl 调试信息 |

### 4.2 常用命令

```bash
# 列出所有活跃话题
rostopic list

# 查看节点列表
rosnode list

# 查看某话题发布频率
rostopic hz /truth_odom      # 应约 50-100 Hz

# 查看某话题的完整定义
rosmsg show quadrotor_msgs/PositionCommand

# 手动触发起飞（调试用）
rostopic pub /px4ctrl/takeoff_land quadrotor_msgs/TakeoffLand \
  "header: {stamp: now}" "takeoff_land_cmd: 1" -1
```

---

## 第五章：常见问题排查

### 5.1 catkin_make 失败

**症状**：`Could not find package 'px4'`

**原因**：未 source 环境脚本

**解决**：
```bash
source ~/XTDrone/cleanroom_ws/tools/source_phase1_env.sh
catkin_make -j4
```

---

**症状**：`TakeoffLand.msg` 不存在

**原因**：使用了外部 quadrotor_msgs 而不是本地版本

**解决**：确认 `src/quadrotor_msgs` 是本地目录（非符号链接）
```bash
ls -la src/quadrotor_msgs/  # 不应显示 "->"
ls src/quadrotor_msgs/msg/TakeoffLand.msg  # 应存在
```

---

### 5.2 启动后无人机不起飞

**排查步骤：**

```bash
# 检查1：MAVROS 连接状态
rostopic echo /mavros/state -n 1
# armed: false, mode: "..." → 正常等待起飞指令
# 如果话题无输出 → MAVROS 未连接 PX4

# 检查2：里程计是否正常
rostopic echo /truth_odom/pose/pose/position -n 1
# 如果无输出 → truth_odom_adapter 未找到 Gazebo 模型

# 检查3：起飞指令是否发出
rostopic echo /px4ctrl/takeoff_land
# 应在 takeoff_delay 秒后收到 takeoff_land_cmd: 1

# 检查4：查看 px4ctrl 日志
rosnode info /px4ctrl
```

---

**症状**：无人机起飞悬停但不飞向目标

**原因**：规划触发器未触发

**排查**：
```bash
# 检查 traj_start_trigger 是否发布
rostopic echo /traj_start_trigger

# 检查 ego_planner 状态
rostopic echo /planning/bspline  # 有输出说明规划器在工作
```

---

### 5.3 无人机飞行不稳定/震荡

可能原因：PID 增益过高。调整 `ctrl_param_fpv.yaml`：

```yaml
gain:
  Kp0: 1.0   # 位置增益（降低以减少超调）
  Kp1: 1.0
  Kp2: 1.0
  Kv0: 1.2   # 速度增益
  Kv1: 1.2
  Kv2: 1.2
```

修改后重新构建并重启：
```bash
catkin_make -j4
roslaunch clean_uav_core phase1_fullstack.launch
```

---

### 5.4 多机场景（Phase 2）

**启动顺序非常重要**，必须严格按以下顺序：

```bash
# 终端1：先启动仿真层（等待 Gazebo 完全加载 10~15s）
roslaunch clean_uav_core phase2_px4_multi_sim.launch

# 终端2（等 iris_0 和 iris_1 都出现在 Gazebo 后）：启动算法层
roslaunch clean_uav_core phase2_dual_uav_stack.launch
```

**常见错误**：反向启动会导致 MAVROS 无法与 PX4 建立连接。

---

## 第六章：开发指南

### 6.1 添加新的 Python 脚本节点

1. 在 `src/clean_uav_core/scripts/` 创建脚本（首行必须是 `#!/usr/bin/env python3`）
2. 日志前缀统一用 `[clean_uav_core]`
3. 在 `CMakeLists.txt` 中注册：

```cmake
catkin_install_python(PROGRAMS
  scripts/your_new_script.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
```

4. 在对应的 launch 文件中添加 `<node>` 标签

### 6.2 添加新的 launch 文件

推荐参考 `phase1_minimal_demo.launch` 的模式：
- 所有话题名用 `<arg>` 参数化
- 多机场景下 MAVROS 话题不要硬编码
- 使用 `if="$(arg use_xxx)"` 控制节点的可选启动

### 6.3 修改 EGO-Planner 参数

EGO 规划参数通过 `phase1_minimal_demo.launch` 的 `<param>` 标签传递，**不要直接修改 EGO 原始 launch 文件**（它是符号链接指向的外部仓库）：

```xml
<!-- 在 phase1_minimal_demo.launch 中直接修改 -->
<param name="optimization/max_vel" value="3.0"/>   <!-- 最大速度 m/s -->
<param name="optimization/max_acc" value="4.0"/>   <!-- 最大加速度 m/s² -->
<param name="fsm/planning_horizon" value="10.0"/>  <!-- 规划距离 m -->
```

### 6.4 理解 px4ctrl 的推力模型标定

当 `accurate_thrust_model: false`（默认）时，使用简单线性映射：

```
F_hover = mass × g
u_hover = hover_percentage（约 0.5~0.6，仿真中为 0.58）
```

如果悬停时无人机缓慢爬升/下降，调整 `hover_percentage`：
- 上升 → 略微降低 `hover_percentage`
- 下降 → 略微提高 `hover_percentage`

---

## 第七章：学习路径建议

如果你想深入理解本系统，建议按以下顺序阅读：

1. **本文档**（你正在读的）— 快速上手
2. [system_architecture_CN.md](system_architecture_CN.md) — 深入理解架构
3. [directory_structure_CN.md](directory_structure_CN.md) — 了解每个文件的职责
4. `src/clean_uav_core/launch/phase1_minimal_demo.launch` — 理解节点连接方式
5. `src/px4ctrl/src/PX4CtrlFSM.cpp_guide.md` — 理解控制器状态机
6. [phase1_px4ctrl_config_explanation_CN.md](phase1_px4ctrl_config_explanation_CN.md) — 深入参数调优
7. [CLEANROOM_PHASE2_MULTI_UAV_PLAN_CN.md](CLEANROOM_PHASE2_MULTI_UAV_PLAN_CN.md) — 多机扩展

---

## 附录：核心依赖版本

| 软件 | 版本 |
|------|------|
| Ubuntu | 20.04 LTS |
| ROS | Noetic (1.15.x) |
| Gazebo | 11.x |
| PX4 Firmware | 基于 XTDrone 定制版（约 v1.11）|
| Python | 3.8（系统自带）|
| Eigen | 3.3.7 |

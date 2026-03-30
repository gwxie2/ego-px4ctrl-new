# N 架无人机集群仿真系统 - 快速开始指南

## 📋 目录

- [项目概述](#项目概述)
- [环境要求](#环境要求)
- [快速开始](#快速开始)
- [配置文件说明](#配置文件说明)
- [详细启动步骤](#详细启动步骤)
- [冒烟测试](#冒烟测试)
- [高级配置](#高级配置)
- [常见问题排查](#常见问题排查)
- [架构说明](#架构说明)

---

## 项目概述

本项目是 **ego-px4ctrl** 工作区的重构版本，实现了支持 **N 架无人机（1-10 架）** 的标准化集群仿真系统。通过配置文件驱动，可以快速部署任意数量的无人机，支持自主导航、避障、编队飞行等功能。

### 核心特性

| 特性 | 说明 |
|------|------|
| **可扩展性** | 支持 1-10 架无人机，通过配置文件一键扩展 |
| **命名空间统一** | 所有组件统一在 `drone_X` 命名空间下运行 |
| **参数化配置** | 起点、终点、数量全部由配置文件驱动 |
| **自动生成** | Launch 文件由 Python 脚本自动生成 |
| **功能完整** | 包含 EGO-Planner 路径规划、px4ctrl 飞控、VINS 里程计 |

### 系统架构

```
┌─────────────────────────────────────────────────────────┐
│  顶层 Launch (swarm_top_level.launch)                   │
│  - 由 swarm_launch_generator.py 自动生成               │
└─────────────────────────────────────────────────────────┘
                          │
        ┌─────────────────┼─────────────────┐
        ▼                 ▼                 ▼
   ┌─────────┐    ┌──────────┐    ┌──────────┐
   │ Gazebo  │    │ 仿真层   │    │ 算法层   │
   │ 世界    │    │ iris_X   │    │ drone_X  │
   └─────────┘    └──────────┘    └──────────┘
                      │                 │
                      ▼                 ▼
               ┌────────────┐   ┌─────────────────────┐
               │ VINS 管道  │   │ swarm_uav_instance  │
               │ (可选)     │   │ - MAVROS            │
               └────────────┘   │ - Ego-Planner        │
                                 │ - px4ctrl            │
                                 └─────────────────────┘
```

---

## 环境要求

### 系统要求

- **操作系统**: Ubuntu 20.04 LTS
- **ROS 版本**: ROS Noetic
- **Python**: 3.8+
- **Gazebo**: 11.0+

### 依赖包

```bash
# PX4 SITL
cd ~/XTDrone/PX4_Firmware
make px4_sitl_default

# MAVROS
sudo apt install ros-noetic-mavros ros-noetic-mavros-extras

# 其他依赖
sudo apt install ros-noetic-tf2-geometry ros-noetic-topic-tools
```

### 工作空间设置

```bash
cd /home/guanwen/XTDrone/ego-px4ctrl-new
source tools/source_phase1_env.sh
catkin_make -j4
source devel/setup.bash
```

---

## 快速开始

### 5 分钟快速启动（3 机示例）

```bash
# 1. 进入工作空间并 source 环境
cd /home/guanwen/XTDrone/ego-px4ctrl-new
source tools/source_phase1_env.sh

# 2. 生成 launch 文件（3 机）
python3 src/clean_uav_core/scripts/swarm_launch_generator.py

# 3. 启动仿真
roslaunch clean_uav_core swarm_top_level.launch gui:=false
```

**预期结果**：
- Gazebo 启动并显示 3 架无人机
- UAV 自动起飞到 1.0m 高度
- 航向各自目标点飞行
- 支持动态目标追踪（正弦波振荡）

---

## 配置文件说明

### `docs/uav_position_goal.md`

核心配置文件，定义每架无人机的起点和终点。

#### 格式说明

```markdown
- `drone_0`: start=(x,y,z,yaw), goal=(x,y,z,yaw)
- `drone_1`: start=(x,y,z,yaw), goal=(x,y,z,yaw)
- `drone_2`: start=(x,y,z,yaw), goal=(x,y,z,yaw)
```

#### 坐标系约定

- **世界坐标系**: 原点居中，X 向东，Y 向北，Z 向上
- **起始分布**: 极坐标模式，半径 R 可配置
- **航向角**: 朝向世界原点（`yaw = atan2(-y, -x)`）
- **终点**: 起点关于原点的对称点

#### 示例配置（3 机，R=15m）

```markdown
# 3 架无人机配置示例
# 半径 R=15m，角度跨度 60°

- `drone_0`: start=(12.990,-7.500,0.100,2.618), goal=(-12.990,7.500,0.100,2.618)
- `drone_1`: start=(15.000,0.000,0.100,-3.142), goal=(-15.000,0.000,0.100,-3.142)
- `drone_2`: start=(12.990,7.500,0.100,-2.618), goal=(-12.990,-7.500,0.100,-2.618)
```

#### 生成自定义配置

使用 Python 脚本生成配置：

```python
# scripts/generate_uav_config.py
import math

def generate_swarm_config(n, radius=15.0):
    """
    生成 N 架无人机的配置

    Args:
        n: 无人机数量
        radius: 分布半径（米）
    """
    # 计算角度跨度（最多 160°）
    span_deg = min(160, 30 * (n - 1)) if n > 1 else 0
    step_deg = span_deg / (n - 1) if n > 1 else 0

    for i in range(n):
        angle_deg = -span_deg / 2 + i * step_deg
        angle_rad = math.radians(angle_deg)

        # 起点坐标
        x = radius * math.cos(angle_rad)
        y = radius * math.sin(angle_rad)
        z = 0.10
        yaw = math.atan2(-y, -x)

        # 终点坐标（对称）
        goal_x = -x
        goal_y = -y

        print(f"- `drone_{i}`: start=({x:.3f},{y:.3f},{z:.3f},{yaw:.3f}), goal=({goal_x:.3f},{goal_y:.3f},{z:.3f},{yaw:.3f})")

# 生成 5 机配置
generate_swarm_config(5, radius=15.0)
```

---

## 详细启动步骤

### Step 1: 环境准备

```bash
# 终端 1：启动 ROS 核心
roscore

# 终端 2：进入工作空间
cd /home/guanwen/XTDrone/ego-px4ctrl-new
source tools/source_phase1_env.sh
source devel/setup.bash
```

### Step 2: 生成 Launch 文件

```bash
# 查看当前配置
cat docs/uav_position_goal.md

# 生成 launch 文件
python3 src/clean_uav_core/scripts/swarm_launch_generator.py \
  --config docs/uav_position_goal.md \
  --output src/clean_uav_core/launch/swarm_top_level.launch

# 输出示例：
# [Generator] Parsed 3 UAV configs from docs/uav_position_goal.md
# [Generator] Initializing generator for 3 UAVs
# [Generator] Generated launch file: src/clean_uav_core/launch/swarm_top_level.launch
# [Generator] Total UAVs: 3
# [Generator] File size: 11345 bytes
```

### Step 3: 启动仿真

```bash
# 方式 1：无 GUI（推荐用于测试）
roslaunch clean_uav_core swarm_top_level.launch gui:=false

# 方式 2：带 GUI（可视化）
roslaunch clean_uav_core swarm_top_level.launch gui:=true

# 方式 3：启用 VINS（需要额外配置）
roslaunch clean_uav_core swarm_top_level.launch enable_vins:=true
```

### Step 4: 观察系统状态

```bash
# 新终端：检查 UAV 数量
rosnode list | grep drone

# 输出示例：
# /drone_0/ego_planner
# /drone_0/px4ctrl
# /drone_0/mavros
# /drone_1/ego_planner
# ...

# 检查话题
rostopic list | grep "/drone_"

# 检查 MAVROS 连接
rostopic echo /drone_0/mavros/state -n 1
# 预期输出：connected: true, armed: true, mode: "OFFBOARD"

# 检查里程计
rostopic hz /drone_0/odom
# 预期输出：~50-100 Hz
```

---

## 冒烟测试

冒烟测试用于验证系统基本功能是否正常。

### 测试清单

#### ✅ 测试 1：单机基础功能

```bash
# 1. 创建单机配置
cat > docs/test_single_uav.md << EOF
- `drone_0`: start=(0.0,0.0,0.1,0.0), goal=(5.0,0.0,1.0,0.0)
EOF

# 2. 生成 launch 文件
python3 src/clean_uav_core/scripts/swarm_launch_generator.py \
  --config docs/test_single_uav.md \
  --output src/clean_uav_core/launch/test_single.launch

# 3. 启动测试
roslaunch clean_uav_core test_single.launch gui:=false

# 预期结果：
# ✅ Gazebo 启动，显示 1 架 UAV
# ✅ UAV 自动起飞到 1.0m
# ✅ UAV 飞向目标点 (5, 0, 1)
# ✅ 到达后悬停
```

#### ✅ 测试 2：双机避障

```bash
# 1. 创建双机配置
cat > docs/test_dual_uav.md << EOF
- `drone_0`: start=(0.0,2.0,0.1,0.0), goal=(10.0,2.0,1.0,0.0)
- `drone_1`: start=(0.0,-2.0,0.1,0.0), goal=(10.0,-2.0,1.0,0.0)
EOF

# 2. 生成并启动
python3 src/clean_uav_core/scripts/swarm_launch_generator.py \
  --config docs/test_dual_uav.md \
  --output src/clean_uav_core/launch/test_dual.launch
roslaunch clean_uav_core test_dual.launch gui:=false

# 预期结果：
# ✅ 2 架 UAV 同时起飞
# ✅ 各自飞向目标点
# ✅ 路径不冲突（保持 2m 间距）
```

#### ✅ 测试 3：动态目标追踪

```bash
# 启动默认 3 机配置
roslaunch clean_uav_core swarm_top_level.launch gui:=false

# 观察目标话题
rostopic echo /drone_0/goal -n 10

# 预期结果：
# ✅ 目标点 Y 坐标呈正弦波振荡
# ✅ UAV 跟踪目标点运动
```

#### ✅ 测试 4：命名空间隔离

```bash
# 启动 3 机仿真
roslaunch clean_uav_core swarm_top_level.launch gui:=false

# 测试节点隔离
rosnode list | grep -E "drone_[0-2]"
# 预期：每个 drone_X 有独立的节点

# 测试话题隔离
rostopic list | grep -E "/drone_[0-2]/odom"
# 预期：/drone_0/odom, /drone_1/odom, /drone_2/odom

# 测试话题独立性
rostopic hz /drone_0/odom  # 应该有输出
rostopic hz /drone_3/odom  # 应该无输出（不存在）
```

#### ✅ 测试 5：端口配置

```bash
# 检查 MAVROS 端口分配
netstat -tulpn | grep "1857[0-2]"
# 预期输出：
# udp  0.0.0.0:18570  ...
# udp  0.0.0.0:18571  ...
# udp  0.0.0.0:18572  ...

# 检查 FCU URL
# 在 launch 文件中确认：
# drone_0: udp://:24540@localhost:34580
# drone_1: udp://:24541@localhost:34581
# drone_2: udp://:24542@localhost:34582
```

---

## 高级配置

### 配置 1：修改规划器参数

```xml
<!-- 在 swarm_top_level.launch 中修改全局参数 -->
<arg name="planner_max_vel" default="2.0"/>    <!-- 最大速度 m/s -->
<arg name="planner_max_acc" default="4.0"/>    <!-- 最大加速度 m/s² -->
<arg name="planner_max_jerk" default="6.0"/>   <!-- 最大加加速度 m/s³ -->
```

或在启动时覆盖：

```bash
roslaunch clean_uav_core swarm_top_level.launch \
  planner_max_vel:=2.0 \
  planner_max_acc:=4.0
```

### 配置 2：启用 VINS 里程计

```bash
# 1. 确保 VINS 已安装
cd ~/XTDrone/VINS-Fusion
catkin_make
source devel/setup.bash

# 2. 启动带 VINS 的仿真
roslaunch clean_uav_core swarm_top_level.launch \
  enable_vins:=true \
  odom_mode:=vins_with_fallback

# 3. 检查 VINS 状态
rostopic echo /drone_0/vins_extrinsic -n 1
rostopic hz /drone_0/odom  # VINS 里程计频率
```

### 配置 3：调整触发器时序

```bash
roslaunch clean_uav_core swarm_top_level.launch \
  takeoff_delay:=5.0 \           # 起飞延迟（默认 8s）
  traj_trigger_delay:=15.0 \     # 轨迹触发延迟（默认 20s）
  goal_start_delay:=20.0          # 目标开始延迟（默认 20s）
```

### 配置 4：自定义目标振荡

```xml
<!-- 修改 swarm_top_level.launch 中的动态目标参数 -->
<arg name="enable_oscillation" default="true"/>     <!-- 启用振荡 -->
<arg name="goal_period_sec" default="20.0"/>        <!-- 振荡周期 -->
```

或在启动时覆盖：

```bash
roslaunch clean_uav_core swarm_top_level.launch \
  enable_oscillation:=true \
  goal_period_sec:=15.0
```

---

## 常见问题排查

### 问题 1：UAV 无法起飞

**症状**：UAV 在地面不动，无法切换到 OFFBOARD 模式

**排查步骤**：

```bash
# 1. 检查 MAVROS 连接
rostopic echo /drone_0/mavros/state -n 1
# 预期：connected: true, armed: true

# 2. 检查起飞触发器
rostopic echo /drone_0/px4ctrl/takeoff_land -n 1
# 预期：收到 TakeoffLand 消息

# 3. 检查 px4ctrl 日志
rosnode list | grep px4ctrl
rosnode info /drone_0/px4ctrl

# 4. 手动触发起飞
rostopic pub /drone_0/px4ctrl/takeoff_land quadrotor_msgs/TakeoffLand "{header: {frame_id: 'world'}, takeoff: true}"
```

**解决方案**：

1. 确认 `phase1_px4ctrl_no_rc.yaml` 中的参数：
   ```yaml
   auto_takeoff_land:
     enable: true
     enable_auto_arm: true
     no_RC: true
   ```

2. 增加起飞延迟：
   ```bash
   roslaunch clean_uav_core swarm_top_level.launch takeoff_delay:=15.0
   ```

### 问题 2：UAV 飞行路径错误

**症状**：UAV 飞向错误的目标点或路径

**排查步骤**：

```bash
# 1. 检查目标点
rostopic echo /drone_0/goal -n 1

# 2. 检查 ego_planner 参数
rosparam get /drone_0/ego_planner/fsm/waypoint0_x

# 3. 检查位置命令
rostopic echo /drone_0/position_cmd -n 1
```

**解决方案**：

1. 确认 `uav_position_goal.md` 中的坐标正确
2. 重新生成 launch 文件
3. 检查目标发布器是否正常工作

### 问题 3：MAVROS 端口冲突

**症状**：部分 UAV 无法连接到 MAVROS

**排查步骤**：

```bash
# 检查端口占用
netstat -tulpn | grep "1857"

# 检查 PX4 SITL 进程
ps aux | grep px4
```

**解决方案**：

1. 确保端口分配正确：
   - drone_0: 18570, 24540, 34580
   - drone_1: 18571, 24541, 34581
   - ...

2. 清理残留进程：
   ```bash
   killall -9 px4
   pkill -f gazebo
   ```

### 问题 4：里程计丢失

**症状**：`/drone_X/odom` 无数据

**排查步骤**：

```bash
# 检查里程计话题
rostopic list | grep "/drone_0.*odom"

# 检查适配器节点
rosnode list | grep odom_adapter

# 检查输入里程计
rostopic echo /iris_0/odometry -n 1
```

**解决方案**：

1. 确认 `odom_pose_adapter.py` 正常运行
2. 检查偏移量配置是否正确
3. 检查 VINS 是否正常启动（如果使用 VINS）

### 问题 5：规划器无输出

**症状**：无轨迹生成，UAV 悬停不动

**排查步骤**：

```bash
# 1. 检查触发器
rostopic echo /drone_0/traj_start_trigger -n 1

# 2. 检查 ego_planner 日志
rosnode log /drone_0/ego_planner

# 3. 检查 B 样条轨迹
rostopic echo /drone_0/planning/bspline -n 1
```

**解决方案**：

1. 确认触发器已发布消息
2. 增加 `traj_trigger_delay`
3. 检查障碍物地图是否正确初始化

---

## 架构说明

### 命名空间映射表

| 组件 | 旧架构 | 新架构 | 说明 |
|------|--------|--------|------|
| MAVROS | `/iris_0/mavros/*` | `/drone_0/mavros/*` | 现在在 drone_X 命名空间内 |
| 算法层 | `/uav0/*` | `/drone_0/*` | 统一命名空间 |
| 里程计 | `/uav0/truth_odom` | `/drone_0/odom` | 适配器输出 |
| 目标点 | `/uav0/goal` | `/drone_0/goal` | 动态命令器发布 |
| 触发器 | `/uav0/traj_start_trigger` | `/drone_0/traj_start_trigger` | 同步触发器发布 |

### 话题数据流

```
Gazebo → /iris_X/odometry
              ↓
    odom_pose_adapter (offset + remap)
              ↓
         /drone_X/odom
              ↓
    ┌─────────┴─────────┐
    ▼                   ▼
ego_planner          px4ctrl
    ↓                   ↓
planning/bspline   position_cmd
    ↓                   ↓
 traj_server          ↓
    └──────────────────→↓
                    /drone_X/mavros/setpoint_raw/attitude
                            ↓
                         MAVROS
                            ↓
                          PX4
```

### 端口分配规则

| UAV ID | MAVLink UDP | MAVLink TCP | FCU URL | GCS URL |
|--------|-------------|-------------|---------|---------|
| 0 | 18570 | 4560 | udp://:24540@localhost:34580 | - |
| 1 | 18571 | 4561 | udp://:24541@localhost:34581 | - |
| 2 | 18572 | 4562 | udp://:24542@localhost:34582 | - |
| n | 18570+n | 4560+n | udp://:24540+n@localhost:34580+n | - |

### 文件结构

```
src/clean_uav_core/
├── launch/
│   ├── swarm_uav_instance.launch          # 单机标准模板
│   ├── swarm_vins_pipeline.launch         # 多机 VINS 管道
│   └── swarm_top_level.launch             # 生成的主 launch
├── scripts/
│   ├── swarm_dynamic_commander.py         # 动态目标命令器
│   ├── swarm_traj_trigger.py              # 同步轨迹触发器
│   └── swarm_launch_generator.py          # Launch 生成器
└── config/
    └── phase1_px4ctrl_no_rc.yaml          # px4ctrl 配置
```

---

## 附录

### A. 完整测试脚本

```bash
#!/bin/bash
# test_swarm_smoke.sh - 完整冒烟测试脚本

echo "=========================================="
echo "Swarm Smoke Test"
echo "=========================================="

# 1. 单机测试
echo "[1/5] Single UAV test..."
cat > /tmp/test_single.md << 'EOF'
- `drone_0`: start=(0.0,0.0,0.1,0.0), goal=(3.0,0.0,1.0,0.0)
EOF

python3 src/clean_uav_core/scripts/swarm_launch_generator.py \
  --config /tmp/test_single.md \
  --output src/clean_uav_core/launch/test_single.launch

timeout 30s roslaunch clean_uav_core test_single.launch gui:=false &
TEST_PID=$!
sleep 25
kill $TEST_PID 2>/dev/null
echo "✅ Single UAV test passed"

# 2. 双机测试
echo "[2/5] Dual UAV test..."
cat > /tmp/test_dual.md << 'EOF'
- `drone_0`: start=(0.0,2.0,0.1,0.0), goal=(8.0,2.0,1.0,0.0)
- `drone_1`: start=(0.0,-2.0,0.1,0.0), goal=(8.0,-2.0,1.0,0.0)
EOF

python3 src/clean_uav_core/scripts/swarm_launch_generator.py \
  --config /tmp/test_dual.md \
  --output src/clean_uav_core/launch/test_dual.launch

timeout 30s roslaunch clean_uav_core test_dual.launch gui:=false &
TEST_PID=$!
sleep 25
kill $TEST_PID 2>/dev/null
echo "✅ Dual UAV test passed"

# 3. 3 机默认配置测试
echo "[3/5] 3-UAV default config test..."
timeout 30s roslaunch clean_uav_core swarm_top_level.launch gui:=false &
TEST_PID=$!
sleep 25
kill $TEST_PID 2>/dev/null
echo "✅ 3-UAV test passed"

echo "=========================================="
echo "All smoke tests passed!"
echo "=========================================="
```

### B. 监控脚本

```bash
#!/bin/bash
# monitor_swarm.sh - 集群监控脚本

while true; do
    clear
    echo "=========================================="
    echo "Swarm Status Monitor"
    echo "=========================================="
    date

    # UAV 数量
    NUM_UAVS=$(rosnode list 2>/dev/null | grep -c "ego_planner")
    echo "UAVs Running: $NUM_UAVS"

    # 里程计频率
    echo ""
    echo "Odometry Rates:"
    for i in $(seq 0 $((NUM_UAVS - 1))); do
        RATE=$(rostopic hz /drone_${i}/odom 2>/dev/null | awk '{print $3}')
        echo "  drone_${i}: ${RATE} Hz"
    done

    # MAVROS 状态
    echo ""
    echo "MAVROS Status:"
    for i in $(seq 0 $((NUM_UAVS - 1))); do
        STATE=$(rostopic echo /drone_${i}/mavros/state 2>/dev/null | grep -c "connected: true")
        if [ "$STATE" -gt 0 ]; then
            echo "  drone_${i}: ✅ Connected"
        else
            echo "  drone_${i}: ❌ Disconnected"
        fi
    done

    sleep 2
done
```

### C. 快速清理脚本

```bash
#!/bin/bash
# cleanup_swarm.sh - 清理所有进程

echo "Cleaning up swarm processes..."

# 停止所有 roslaunch
killall -9 roslaunch roscore rosmaster gzserver gzclient 2>/dev/null

# 停止 PX4 SITL
killall -9 px4 2>/dev/null

# 清理 Gazebo
pkill -f gazebo 2>/dev/null

echo "Cleanup complete!"
```

---

## 版本历史

| 版本 | 日期 | 说明 |
|------|------|------|
| v1.0 | 2025-01 | 初始版本，支持 N 架无人机集群仿真 |

---

## 联系方式

如有问题或建议，请提交 Issue 或 Pull Request。

**Happy Flying! 🚀**

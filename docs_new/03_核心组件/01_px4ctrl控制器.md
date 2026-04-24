# px4ctrl 控制器详解

## 组件概述

px4ctrl 是系统的高层位置控制器，负责接收轨迹指令并转换为姿态控制指令发送给 PX4 飞控。

## 核心文件结构

```
src/px4ctrl/
├── src/
│   ├── px4ctrl_node.cpp       # ROS 节点入口
│   ├── PX4CtrlFSM.cpp         # 有限状态机实现
│   ├── controller.cpp         # 级联 PID 控制器
│   ├── input.cpp              # 输入数据处理
│   └── PX4CtrlParam.cpp       # 参数加载
├── include/px4ctrl/
│   ├── PX4CtrlFSM.h
│   ├── controller.h
│   ├── input.h
│   └── PX4CtrlParam.h
└── config/
    └── ctrl_param_fpv.yaml    # 主配置文件
```

## 有限状态机（FSM）

px4ctrl 使用五态模型管理飞行状态：

```
     ┌─────────────────┐
     │  MANUAL_CTRL    │ ← RC 控制模式（FSM 不介入）
     └────────┬────────┘
              │ 解锁 + Offboard
              ▼
     ┌─────────────────┐
     │  AUTO_TAKEOFF   │ ← 收到 TakeoffLand.TAKEOFF
     └────────┬────────┘
              │ 到达目标高度
              ▼
     ┌─────────────────┐
     │  AUTO_HOVER     │ ← 等待 PositionCommand
     └────────┬────────┘
              │ 收到 PositionCommand
              ▼
     ┌─────────────────┐
     │   CMD_CTRL      │ ← 执行轨迹跟踪（主工作状态）
     └────────┬────────┘
              │ 收到 TakeoffLand.LAND
              ▼
     ┌─────────────────┐
     │   AUTO_LAND     │ ← 下降并检测落地
     └─────────────────┘
```

### 状态说明

| 状态 | 功能 | 输入 | 输出 |
|------|------|------|------|
| **MANUAL_CTRL** | RC 直接控制 | RC 信号 | 不发布姿态指令 |
| **AUTO_TAKEOFF** | 自动起飞 | TakeoffLand.TAKEOFF | 悬停姿态指令 |
| **AUTO_HOVER** | 悬停等待 | 里程计 | 当前悬停姿态 |
| **CMD_CTRL** | 轨迹跟踪 | PositionCommand | 跟踪姿态指令 |
| **AUTO_LAND** | 自动降落 | TakeoffLand.LAND | 下降姿态指令 |

### 关键状态转换

```cpp
// AUTO_TAKEOFF → AUTO_HOVER
if (state.hover_timeout_reached() && pos_error < threshold) {
    state.transition(AUTO_HOVER);
}

// AUTO_HOVER → CMD_CTRL
if (received_position_cmd && offboard_mode) {
    state.transition(CMD_CTRL);
}

// CMD_CTRL → AUTO_LAND
if (received_land_cmd) {
    state.transition(AUTO_LAND);
}
```

## 级联 PID 控制器

### 控制结构

```
目标位置 p_des
    │
    │ 位置环 PID
    │   u_p = Kp * (p_des - p)
    ▼
目标速度 v_des
    │
    │ 速度环 PID
    │   u_v = Kv * (v_des - v)
    ▼
目标加速度 a_des
    │
    │ 重力补偿 + 推力模型
    ▼
姿态 q_des + 推力 thrust
    │
    ▼
AttitudeTarget → MAVROS → PX4
```

### PID 参数说明

```yaml
gain:
  # 位置环增益
  Kp0: 1.5   # X 轴
  Kp1: 1.5   # Y 轴
  Kp2: 1.5   # Z 轴

  # 速度环增益
  Kv0: 1.5   # X 轴
  Kv1: 1.5   # Y 轴
  Kv2: 1.5   # Z 轴
```

**调参原则**：
- 位置增益 `Kp` 决定响应速度，过大会导致超调和震荡
- 速度增益 `Kv` 影响阻尼，通常与 `Kp` 协同调整
- 建议先调整 Z 轴（高度），再调整 X/Y 轴

## 推力模型

### 简单线性模型（默认）

```yaml
thrust_model:
  accurate_thrust_model: false
  hover_percentage: 0.58  # 悬停油门百分比
```

推力计算：
```
F = mass * g * (thrust / hover_percentage)
```

### 精确推力模型

```yaml
thrust_model:
  accurate_thrust_model: true
  K1: 0.7583   # 需要标定！
  K2: 1.6942   # 需要标定！
  K3: 0.6786   # 需要标定！
```

推力计算：
```
F = K1 * V^k2 * (K3 * u^2 + (1-K3) * u)
```

其中：
- `V`：电池电压
- `u`：归一化推力信号 [0, 1]

**标定方法**：
1. 悬停时记录电池电压和油门值
2. 多次飞行采集数据
3. 使用最小二乘法拟合 K1, K2, K3

## 关键参数配置

### 基础参数

```yaml
# 物理参数
mass: 1.2              # 机体质量 (kg)
gra: 9.81              # 重力加速度 (m/s²)

# 控制频率
ctrl_freq_max: 100.0   # 最大控制频率 (Hz)

# 角度限制
max_angle: 30          # 最大倾斜角 (度)
```

### 自动起降参数

```yaml
auto_takeoff_land:
  enable: true              # 启用自动起降
  enable_auto_arm: true     # 允许软件解锁
  no_RC: false              # 是否无遥控器模式
  takeoff_height: 1.0       # 起飞目标高度 (m)
  takeoff_land_speed: 0.3   # 起降速度 (m/s)
```

### 超时参数

```yaml
msg_timeout:
  odom: 0.5    # 里程计超时 (s)
  rc: 0.5      # RC 超时 (s)
  cmd: 0.5     # 指令超时 (s)
  imu: 0.5     # IMU 超时 (s)
  bat: 0.5     # 电池超时 (s)
```

## 输入消息

### 订阅的话题

| 话题 | 消息类型 | 说明 |
|------|----------|------|
| `~odom` | `nav_msgs/Odometry` | 里程计输入 |
| `~cmd` | `quadrotor_msgs/PositionCommand` | 位置指令 |
| `/mavros/state` | `mavros_msgs/State` | PX4 状态 |
| `/mavros/imu/data` | `sensor_msgs/Imu` | IMU 数据 |
| `~rc` | `mavros_msgs/RCIn` | RC 输入（可选） |
| `~takeoff_land` | `quadrotor_msgs/TakeoffLand` | 起降指令 |

## 输出消息

### 发布的话题

| 话题 | 消息类型 | 说明 |
|------|----------|------|
| `/mavros/setpoint_raw/attitude` | `mavros_msgs/AttitudeTarget` | 姿态指令 |
| `/px4ctrl/debug` | `quadrotor_msgs/Px4ctrlDebug` | 调试信息 |

### AttitudeTarget 消息结构

```
Header header
uint32 type_mask           # 忽略位（通常=0）
geometry_msgs/Quaternion orientation  # 目标姿态
BodyRate thrust_body_rate  # 角速率（可选）
float64 thrust             # 归一化推力 [0, 1]
```

## 调试技巧

### 查看控制器状态

```bash
# 查看调试信息
rostopic echo /px4ctrl/debug

# 查看当前状态
rostopic echo /px4ctrl/debug | grep state
```

### 常见问题排查

| 问题 | 可能原因 | 解决方法 |
|------|----------|----------|
| 悬停时缓慢上升 | hover_percentage 过高 | 降低 hover_percentage |
| 悬停时缓慢下降 | hover_percentage 过低 | 提高 hover_percentage |
| 飞行震荡 | PID 增益过高 | 降低 Kp/Kv |
| 响应慢 | PID 增益过低 | 提高 Kp/Kv |
| 不能起飞 | enable_auto_arm=false | 设置为 true |

### 性能监控

```bash
# 监控控制频率
rostopic hz /mavros/setpoint_raw/attitude

# 监控控制误差
rostopic echo /px4ctrl/debug | grep pos_err
```

## 参考文档

- **参数详解**：`01_系统概述/05_关键配置文件.md`
- **调参指南**：`09_故障排除/01_控制器调参.md`
- **系统架构**：`01_系统概述/02_系统架构.md`

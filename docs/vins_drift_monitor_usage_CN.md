# VINS 漂移监控工具使用说明

## 概述

`vins_drift_monitor.py` 是一个用于监控 Phase 4 双机系统中 VINS 里程计漂移的独立工具。它实时对比 VINS 里程计与 Gazebo 真值位置，并提供数据记录和可视化功能。

## 功能特性

1. **双机同时监控**：同时监控 `iris_0` 和 `iris_1` 的 VINS 漂移
2. **真值来源**：从 `/gazebo/model_states` 获取无人机真实位置
3. **实时 CSV 记录**：保存位置数据、漂移误差和姿态信息
4. **交互式可视化**：matplotlib 实时显示轨迹和漂移
5. **定期图像保存**：自动保存静态图像到输出目录

## 订阅话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `/gazebo/model_states` | `gazebo_msgs/ModelStates` | Gazebo 真值（所有模型状态） |
| `/iris_0/vins_estimator/imu_propagate` | `nav_msgs/Odometry` | UAV0 的 VINS 里程计 |
| `/iris_1/vins_estimator/imu_propagate` | `nav_msgs/Odometry` | UAV1 的 VINS 里程计 |

## 输出文件

所有输出文件保存在 `/tmp/vins_drift_monitor/` 目录下：

### CSV 文件
- `uav0_vins_drift_YYYYMMDD_HHMMSS.csv` - UAV0 数据记录
- `uav1_vins_drift_YYYYMMDD_HHMMSS.csv` - UAV1 数据记录

CSV 包含以下字段：
```
timestamp_ros, timestamp_wall,
vins_x, vins_y, vins_z,
truth_x, truth_y, truth_z,
drift_x, drift_y, drift_xy,
vins_qx, vins_qy, vins_qz, vins_qw,
truth_qx, truth_qy, truth_qz, truth_qw
```

### 图像文件
- `vins_drift_plot_YYYYMMDD_HHMMSS.png` - 当前漂移状态图（定期更新）

## 可配置参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `~output_dir` | `/tmp/vins_drift_monitor` | 输出目录路径 |
| `~csv_save_interval` | `1.0` | CSV 保存间隔（秒） |
| `~plot_save_interval` | `5.0` | 图像保存间隔（秒） |
| `~history_len` | `1000` | 保留的历史数据点数 |
| `~uav0_model` | `iris_0` | Gazebo 中 UAV0 的模型名称 |
| `~uav1_model` | `iris_1` | Gazebo 中 UAV1 的模型名称 |

## 使用方法

### 方法 1：直接运行（推荐用于调试）

```bash
# 终端 1：启动 Phase 4 仿真
cd /home/guanwen/XTDrone/ego-px4ctrl
source tools/source_phase1_env.sh
roslaunch clean_uav_core phase4_dual_uav_dynamic_goal_stack.launch

# 终端 2：启动漂移监控
cd /home/guanwen/XTDrone/ego-px4ctrl
source tools/source_phase1_env.sh
rosrun clean_uav_core vins_drift_monitor.py
```

### 方法 2：在 launch 文件中集成

在您的 launch 文件中添加：

```xml
<node pkg="clean_uav_core" type="vins_drift_monitor.py"
     name="vins_drift_monitor" output="screen">
  <param name="output_dir" value="/tmp/vins_drift_monitor"/>
  <param name="csv_save_interval" value="1.0"/>
  <param name="plot_save_interval" value="5.0"/>
</node>
```

### 方法 3：自定义配置

```bash
rosrun clean_uav_core vins_drift_monitor.py _output_dir:=/path/to/output _csv_save_interval:=0.5
```

## 可视化界面说明

工具启动后会显示一个包含两个子图的窗口：

### 左图：UAV0 (iris_0)
- **绿色实线**：真值轨迹
- **红色虚线**：VINS 估计轨迹
- **绿色圆点**：当前真值位置
- **红色三角**：当前 VINS 位置
- **蓝色点线**：漂移向量（从真值指向 VINS）
- **文本框**：显示当前/平均/最大漂移量

### 右图：UAV1 (iris_1)
- 内容同左图

## 漂移指标说明

- **drift_x**：VINS 与真值的 X 方向误差
- **drift_y**：VINS 与真值的 Y 方向误差
- **drift_xy**：2D 平面位置误差（欧氏距离）：$\sqrt{drift_x^2 + drift_y^2}$

## 数据分析示例

使用 Python 分析 CSV 数据：

```python
import pandas as pd
import matplotlib.pyplot as plt

# 读取 CSV
df = pd.read_csv('/tmp/vins_drift_monitor/uav0_vins_drift_*.csv')

# 绘制漂移随时间变化
plt.figure(figsize=(12, 6))
plt.plot(df['timestamp_ros'], df['drift_xy'])
plt.xlabel('Time (s)')
plt.ylabel('Drift (m)')
plt.title('VINS Drift Over Time')
plt.grid(True)
plt.show()

# 统计摘要
print(df[['drift_x', 'drift_y', 'drift_xy']].describe())
```

## 故障排查

### 1. 窗口不显示或报错 "No display"

如果在无头环境或远程服务器上运行，设置虚拟显示：

```bash
export DISPLAY=:0
# 或使用 xvfb-run
xvfb-run -a rosrun clean_uav_core vins_drift_monitor.py
```

### 2. Gazebo 模型未找到

如果日志显示 "model 'iris_0' not found"，检查：
- Gazebo 是否正在运行
- 模型名称是否正确（可能因 launch 配置而异）

### 3. VINS 数据未接收

确认：
- VINS 估计器是否正在发布到 `/iris_X/vins_estimator/imu_propagate`
- 使用 `rostopic echo /iris_0/vins_estimator/imu_propagate -n 1` 验证

### 4. CSV 文件为空

- 确认同时收到了真值和 VINS 数据（任一缺失则不记录）
- 检查日志是否有错误信息

## 与 Phase 4 集成

在 Phase 4 中，真值默认不发布到 `/uavX/truth_odom`，因此本工具直接从 `/gazebo/model_states` 获取真值。这确保了即使 `use_truth_odom_runtime=false` 也能进行漂移分析。

如需同时启动 truth_odom_adapter（用于其他节点），可在 launch 中添加：

```xml
<node pkg="clean_uav_core" type="truth_odom_adapter.py" name="uav0_truth_odom_adapter">
  <param name="model_name" value="iris_0"/>
  <param name="odom_topic" value="/uav0/truth_odom"/>
</node>
<node pkg="clean_uav_core" type="truth_odom_adapter.py" name="uav1_truth_odom_adapter">
  <param name="model_name" value="iris_1"/>
  <param name="odom_topic" value="/uav1/truth_odom"/>
</node>
```

# Sensor Degradation Middleware

这个中间件用于在仿真链路里模拟“不是完美传感器”的情况，把 Gazebo 的真值里程计和深度图先做一次降级，再交给现有 planner。目标不是改 planner，而是尽量保持 `odom`、`pose`、`depth_output` 这些接口不变。

## 架构

运行链路如下：

- truth odom -> `sensor_noise_injector.py` -> `odom_` 系列输出 -> `odom_pose_adapter.py` -> `odom` / `pose`
- depth raw -> `depth_float_to_uint16.py` -> `sensor_noise_injector.py` -> `depth_output`

这样做的好处是，planner 侧不用知道是否启用了降级，只需要继续订阅原来的 topic。

## 模式

当前支持三个预设：

- `clean`：基线模式，等价于不开降级。
- `mild`：轻度延迟、轻度漂移、轻度深度噪声和空洞。
- `extreme`：更强的延迟、漂移、量化和深度缺失。

全局开关：

- `sensor_degradation_enabled:=false` 时，odom 直接直通，baseline 不引入额外延迟。
- `sensor_degradation_enabled:=true` 时，按 `sensor_degradation_mode` 选择预设。

## 配置入口

YAML 入口是：

- [src/clean_uav_core/config/swarm_config.yaml](../src/clean_uav_core/config/swarm_config.yaml)
- [src/clean_uav_core/config/swarm_config_10ms.yaml.txt](../src/clean_uav_core/config/swarm_config_10ms.yaml.txt)
- [src/clean_uav_core/config/benchmark_research/stage_a_speed_ramping_v2.yaml](../src/clean_uav_core/config/benchmark_research/stage_a_speed_ramping_v2.yaml)
- [src/clean_uav_core/config/benchmark_research/stage_d_extreme_10ms_v2.yaml](../src/clean_uav_core/config/benchmark_research/stage_d_extreme_10ms_v2.yaml)

生成器会把这些字段展开为顶层 launch 参数，再传给 runtime launch。

## 主要 launch 开关

V1 和 V2 runtime 都支持：

- `sensor_degradation_enabled`
- `sensor_degradation_mode`

在生成后的 top-level launch 中，这两个参数会继续透传到 runtime include。

## 建议的实验方式

1. 先跑 clean baseline，确认 `sensor_degradation_enabled:=false` 下行为不变。
2. 再跑 `mild`，先看 5m/s 的稳定性和重规划频率。
3. 再跑 `extreme`，优先在 10m/s headless 条件下验证是否能维持安全边界。

建议重点看这些指标：

- 重规划成功率
- 最小安全边界
- 控制和规划延迟
- 轨迹抖动和 jerk 累积

## 实验矩阵

这组对照已经单独整理到 [src/clean_uav_core/config/benchmark_research/campaign_matrix_v2.yaml](../src/clean_uav_core/config/benchmark_research/campaign_matrix_v2.yaml) 的 `stage_e`，对应的 launch 文件也已经生成在 [src/clean_uav_core/launch/](../src/clean_uav_core/launch/) 下。

| 速度 | 模式 | 配置基线 | launch 文件 |
| --- | --- | --- | --- |
| 5m/s | clean | `smooth_priority` | [benchmark_stage_e_5p0_clean.launch](../src/clean_uav_core/launch/benchmark_stage_e_5p0_clean.launch) |
| 5m/s | mild | `smooth_priority` | [benchmark_stage_e_5p0_mild.launch](../src/clean_uav_core/launch/benchmark_stage_e_5p0_mild.launch) |
| 5m/s | extreme | `smooth_priority` | [benchmark_stage_e_5p0_extreme.launch](../src/clean_uav_core/launch/benchmark_stage_e_5p0_extreme.launch) |
| 10m/s | clean | `extreme_baseline` | [benchmark_stage_e_10p0_clean.launch](../src/clean_uav_core/launch/benchmark_stage_e_10p0_clean.launch) |
| 10m/s | mild | `extreme_baseline` | [benchmark_stage_e_10p0_mild.launch](../src/clean_uav_core/launch/benchmark_stage_e_10p0_mild.launch) |
| 10m/s | extreme | `extreme_baseline` | [benchmark_stage_e_10p0_extreme.launch](../src/clean_uav_core/launch/benchmark_stage_e_10p0_extreme.launch) |

## 说明

- depth 降级保持向量化处理，不做逐像素 Python 循环。
- odom 关闭降级时走直通，不额外引入队列时延。
- 如果后续需要更强的真实感，可以再加 dropout band、结构化缺失或时序相关噪声。
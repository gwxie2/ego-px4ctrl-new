# Swarm 多机启动架构重构实施计划

## 1. 目标

本次重构目标是把当前基于临时 Python 生成大段 XML 的 swarm 多机启动方案，替换为一套可维护、可迁移、可部署到真实分布式机载电脑的工业级 ROS 架构。

重构后架构应满足以下要求：

- 单机逻辑模板化：单机全栈逻辑集中在 `swarm_uav_instance.launch`
- 顶层负责组装：`swarm_top_level.launch` 仅负责世界、实例、全局调度器、Rviz
- 命名空间统一：对上层 ROS 接口统一为 `drone_X`
- 端口闭环绑定：PX4 Spawn、MAVROS、`fcu_url`、`tgt_system` 由 `drone_id` 数学强绑定
- 去 Relay 化：禁止 `topic_tools/relay`，统一使用 `remap`
- 路径稳健：Python 通过 `rospkg.RosPack().get_path('clean_uav_core')` 定位包路径
- 功能保守迁移：以现有 `phase4_dual_uav_dynamic_goal_stack.launch` 为行为基线，不做无必要功能扩张

## 2. 当前问题复盘

### 2.1 命名空间分裂

当前 swarm 架构存在“底层 `iris_X`，上层 `drone_X`”的双命名空间分裂问题：

- PX4 / MAVROS / Gazebo 偏向 `iris_X`
- planner / px4ctrl / commander 偏向 `drone_X`

这导致：

- MAVROS 与 PX4 实例不稳定对接
- 上层控制链经常依赖额外桥接/Relay
- 启动链难以迁移到真实分布式部署环境

### 2.2 端口逻辑未闭环

当前 `swarm_top_level.launch` 和 `swarm_uav_instance.launch` 存在以下问题：

- Spawn 所用 MAVLink 端口由一处给出
- MAVROS 的 `fcu_url` 由另一处给出
- `tgt_system` 与 `drone_id` 的关系没有在模板内部统一表达

风险是：

- 一旦顶层传参或生成器输出偏离，PX4 与 MAVROS 就会失配

### 2.3 Python 路径解析脆弱

`swarm_launch_generator.py` 目前通过脚本相对路径回溯 workspace，`swarm_dynamic_commander.py` 甚至把 `$(find clean_uav_core)` 当作普通字符串默认值。

这会导致：

- ROS 环境和 shell cwd 不一致时，配置文件找不到
- 迁移到其他机器或其他运行方式时不稳定

### 2.4 Launch 结构职责混乱

当前 `swarm_top_level.launch` 同时承担：

- Gazebo 世界启动
- 多机 PX4 spawn
- 多机 MAVROS
- 多机算法模板 include
- Python 调度器

这意味着顶层知道了太多底层细节，不符合“顶层组装、底层模板自洽”的目标。

### 2.5 VINS 接入方式不合规

当前 `swarm_vins_pipeline.launch` 使用了 `topic_tools/relay`。这与目标架构“禁止 relay、只允许 remap”直接冲突。

## 3. 重构原则

### 3.1 红线

- 禁止继续扩大 `iris_X` 与 `drone_X` 的混合设计
- 禁止继续依赖 `topic_tools/relay`
- 禁止生成器直接拼接底层 Spawn / MAVROS XML 细节
- 禁止 Python 用工作区相对路径猜测配置文件位置
- 禁止用局部补丁式修补延续现有混乱结构

### 3.2 工程原则

- 优先整体替换单一模块，不在旧结构上叠补丁
- 保留现有可工作的 planner / px4ctrl / odom 逻辑，减少行为变化
- 先统一架构，再恢复高级能力
- 单机模板是原子单元，顶层只负责组装

## 4. Git 当前未跟踪文件

当前未跟踪新建文件如下：

- `.github/copilot-instructions.md`
- `README_NEW.md`
- `cleanup_swarm.sh`
- `docs/uav_position_goal.md`
- `generate_swarm_config.py`
- `monitor_swarm.sh`
- `src/clean_uav_core/launch/digest.txt`
- `src/clean_uav_core/launch/swarm_top_level.launch`
- `src/clean_uav_core/launch/swarm_uav_instance.launch`
- `src/clean_uav_core/launch/swarm_vins_pipeline.launch`
- `src/clean_uav_core/scripts/swarm_dynamic_commander.py`
- `src/clean_uav_core/scripts/swarm_launch_generator.py`
- `src/clean_uav_core/scripts/swarm_traj_trigger.py`
- `test_swarm_smoke.sh`

本次重构的核心对象主要是以下 6 个 swarm 文件：

- `src/clean_uav_core/launch/swarm_top_level.launch`
- `src/clean_uav_core/launch/swarm_uav_instance.launch`
- `src/clean_uav_core/launch/swarm_vins_pipeline.launch`
- `src/clean_uav_core/scripts/swarm_dynamic_commander.py`
- `src/clean_uav_core/scripts/swarm_launch_generator.py`
- `src/clean_uav_core/scripts/swarm_traj_trigger.py`

## 5. 目标架构

### 5.1 单机原子模板

`swarm_uav_instance.launch` 负责一架无人机的全部核心链路：

- PX4 Spawn
- MAVROS
- PX4 参数 bootstrap
- 运行时 odom/pose 适配
- Ego-Planner
- traj_server
- px4ctrl
- takeoff trigger
- 可选 VINS 接入点

### 5.2 顶层组装

`swarm_top_level.launch` 只保留三部分：

- `empty_world.launch`
- N 个 `swarm_uav_instance.launch` include
- 全局 Rviz 与 Python 调度器

### 5.3 单一命名空间

目标是对系统上层 ROS 接口彻底统一为：

- `drone_0`
- `drone_1`
- `drone_2`

模板最外层唯一命名空间结构为：

```xml
<group ns="drone_$(arg drone_id)">
  ...
</group>
```

## 6. 分阶段实施 TODO

### Phase 0：建立基线与接口核对

目标：确认成功基线、外部 launch 参数接口、现有安装清单。

执行内容：

1. 核对 `phase4_dual_uav_dynamic_goal_stack.launch` 的实际成功链路
2. 核对 `single_vehicle_spawn_xtd.launch` 的参数签名，特别是：
   - 是否支持 `robotNamespace`
   - 车辆模型名与命名空间的关系
3. 核对 `odom_pose_adapter.py`、`multi_vins_bridge.py` 的接口
4. 核对 `CMakeLists.txt` 的 Python 安装清单

输出：

- 明确哪些参数可以直接复用
- 明确哪些能力需要在模板里改造而非简单搬运

### Phase A：重写 `swarm_uav_instance.launch`

目标：让单机模板成为自洽的原子单元。

必须完成的内容：

1. **命名空间统一**
   - 使用唯一外层 `group ns="drone_$(arg drone_id)"`
   - 所有内部节点优先使用相对话题

2. **端口数学闭环**
   - `mavlink_udp_port = 18570 + id`
   - `mavlink_tcp_port = 4560 + id`
   - `fcu_url = udp://:24540+id@localhost:34580+id`
   - `tgt_system = id + 1`

3. **Spawn 修正**
   - 调用 `single_vehicle_spawn_xtd.launch`
   - 若上游支持，显式传 `robotNamespace=drone_$(arg drone_id)`
   - 不再由顶层生成 spawn 片段

4. **MAVROS 告警修复**
   - 显式设置：
     - `negate_measured_roll=false`
     - `negate_measured_pitch=false`
     - `negate_measured_yaw=false`

5. **planner / px4ctrl 话题对齐**
   - planner 订阅相对话题 `odom`
   - traj_server 输出 `position_cmd`
   - px4ctrl 订阅 `odom` 和 `position_cmd`

6. **去掉不必要的跨命名空间耦合**
   - 清理旧的绝对路径 remap
   - 减少 `iris_X` 显式出现在模板内部的机会

输出：

- 一个结构清晰、参数自洽、接口稳定的 `swarm_uav_instance.launch`

### Phase B：重写 `swarm_launch_generator.py`

目标：让生成器只负责组装，不再负责底层 XML 细节。

必须完成的内容：

1. **路径解析改造**
   - 使用 `rospkg.RosPack().get_path('clean_uav_core')`
   - 不再使用目录回溯猜 workspace 根目录

2. **配置解析器稳健化**
   - 支持 `docs/uav_position_goal.md` 中的 `drone_X` 格式
   - 支持带/不带反引号
   - 增强错误提示

3. **生成器职责收缩**
   - 只生成：
     - 顶层 Gazebo world
     - N 个 `swarm_uav_instance.launch` include
     - 全局 Rviz 与 Python 调度器
   - 不再生成 Spawn / MAVROS / 端口 XML 块

4. **禁止项静态检查**
   - 输出 launch 中不应出现：
     - `topic_tools/relay`
     - 多余的 `iris_` 控制链绝对命名空间

输出：

- 轻量、可维护、只生成组装层的 `swarm_launch_generator.py`

### Phase C：修复 `swarm_dynamic_commander.py`

目标：让动态目标命令器在任何 ROS 启动环境中稳定定位配置文件。

必须完成的内容：

1. 用 `rospkg` 定位 `clean_uav_core` 包路径
2. 正确推导 `docs/uav_position_goal.md`
3. 移除未展开的 `$(find clean_uav_core)` 字符串默认值
4. 保持对 `drone_X/goal` 的目标发布逻辑不变

输出：

- 一个路径解析稳健的 `swarm_dynamic_commander.py`

### Phase D：收敛 `swarm_top_level.launch`

目标：把顶层 launch 变成纯组装层。

必须完成的内容：

1. 删除所有底层 PX4 spawn / MAVROS 细节
2. 只保留：
   - Gazebo world
   - N 个实例模板 include
   - Rviz
   - Python 调度器
   - 必要的全局同步触发器

输出：

- 结构干净的 `swarm_top_level.launch`

### Phase E：清理 `swarm_vins_pipeline.launch`

目标：移除 relay 设计，恢复为 remap 驱动的 VINS 接入方案。

必须完成的内容：

1. 移除 `topic_tools/relay`
2. 保留多机 VINS estimator 启动能力
3. 重新定义 VINS 与各实例模板之间的接口边界

备注：

- 这一步优先保证架构正确，不强求一次恢复所有复杂功能

### Phase F：更新安装与验证

目标：让重构后的 swarm 文件可运行、可安装、可静态校验。

必须完成的内容：

1. 更新 `src/clean_uav_core/CMakeLists.txt`
   - 确保新的 swarm Python 脚本进入安装清单
2. 运行静态检查：
   - XML 语法
   - Python 导入
   - `get_errors`
3. 如可行，进行最小启动验证：
   - 单机
   - 双机

输出：

- 可交付的重构结果与剩余风险说明

## 7. 核心文件的目标状态

### 7.1 `swarm_uav_instance.launch`

重构后应具备：

- 单一 `drone_X` 命名空间
- 端口内部计算闭环
- planner 和 px4ctrl 只依赖相对话题
- MAVROS 告警参数显式设置
- 不再依赖顶层传端口 / fcu_url 作为真值

### 7.2 `swarm_launch_generator.py`

重构后应具备：

- `rospkg` 定位能力
- 只负责读取配置并生成组装层 launch
- 无底层 XML 拼接职责
- 输出结果结构简洁

### 7.3 `swarm_top_level.launch`

重构后应具备：

- 纯组装层结构
- 不直接关心 MAVROS/端口/Spawn 底层细节

## 8. 验证标准

### 8.1 静态标准

- `swarm_uav_instance.launch` 中不存在明显命名空间分裂逻辑
- `swarm_vins_pipeline.launch` 中不存在 `topic_tools/relay`
- `swarm_launch_generator.py` 和 `swarm_dynamic_commander.py` 中不存在目录回溯式路径猜测
- `swarm_top_level.launch` 中不再内嵌底层 Spawn/MAVROS 大段结构

### 8.2 运行标准

- `drone_0/mavros/state` 能正常连接 PX4
- `drone_0/odom` 有效输出
- `drone_0/position_cmd` 正常产生
- px4ctrl 与 planner 链路闭合

## 9. 实施顺序

严格按以下顺序执行，避免再次陷入“局部修补”：

1. 落地本计划文档
2. 核对外部依赖接口
3. 重写 `swarm_uav_instance.launch`
4. 重写 `swarm_launch_generator.py`
5. 修复 `swarm_dynamic_commander.py`
6. 收敛 `swarm_top_level.launch`
7. 清理 `swarm_vins_pipeline.launch`
8. 更新安装与验证

## 10. 备注

本计划是后续实施的单一事实源。若执行过程中发现：

- 外部依赖 launch 参数接口与预期不一致
- PX4/Gazebo 上游不允许完全消除某些 `iris_X` 物理模型痕迹
- VINS 接入存在新的约束

则应先更新本文件，再继续实施，避免上下文漂移。
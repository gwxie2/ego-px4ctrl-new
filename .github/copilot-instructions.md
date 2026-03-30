# ego-px4ctrl-new 工作区指令

## 仓库定位

这是一个 ROS Noetic 的 catkin 工作区，用于把 EGO-Planner 的 3D 轨迹规划能力与 px4ctrl 的高层飞控能力集成到 PX4 多旋翼仿真/验证流程中。当前仓库同时包含阶段式启动链路与集群化启动脚本，核心胶水层由 `clean_uav_core` 提供。

## 目录结构

- `src/`：ROS package 源码区，包含规划、控制、消息和工具包。
- `docs/`：中文技术文档，优先作为事实来源，避免在代码注释或新文档中重复大段说明。
- `tools/`：环境引导、日志分析、验证脚本。
- 根目录脚本：`generate_swarm_config.py`、`test_swarm_smoke.sh`、`monitor_swarm.sh`、`cleanup_swarm.sh`，用于集群配置、冒烟测试、监控和清理。
- `build/`、`devel/`：catkin 生成产物，禁止手工编辑。

## `src/` 下的核心包

- `clean_uav_core/`：启动编排、参数覆盖、Python 辅助节点，是这个工作区的胶水层。
- `px4ctrl/`：飞控核心，负责状态机、级联 PID、推力模型与无遥控模式。
- `ego_planner/`：三维轨迹规划与 B 样条优化。
- `quadrotor_msgs/`：本地消息包，必须使用仓库内副本，不能替换为系统 apt 包。
- `plan_env/`：环境与栅格地图表示。
- `path_searching/`：路径搜索，通常作为规划初值。
- `bspline_opt/`：B 样条优化。
- `traj_utils/`：轨迹工具与 trajectory server 相关实现。
- `uav_utils/`：无人机通用工具库。
- `cmake_utils/`：CMake 辅助模块。

## 构建与运行约定

- 每个新终端在执行 `catkin_make`、`roslaunch` 或任何 ROS 相关命令前，先执行 `source tools/source_phase1_env.sh`。
- 常规编译命令是 `catkin_make -j4`，单包编译优先使用 `catkin_make --pkg <package>`。
- 读写代码时优先改 `src/` 和 `tools/` 下的源文件，不要碰 `build/`、`devel/`、`__pycache__/` 之类的生成内容。
- Python 脚本保持 `#!/usr/bin/env python3` 风格，新增脚本时遵循现有日志前缀和参数风格。

## 运行入口

- 单机全栈：优先看 `src/clean_uav_core/launch/phase1_fullstack.launch`。
- 单机拆层：`phase1_px4_sim.launch`、`phase1_algo_stack.launch`、`phase1_px4ctrl_stack.launch`。
- 多机和集群：优先使用 `generate_swarm_config.py` + `swarm_top_level.launch` 这一条配置驱动流程；相应验证脚本是 `test_swarm_smoke.sh`。
- 监控与清理：分别使用 `monitor_swarm.sh` 和 `cleanup_swarm.sh`。
- 如果你修改了多机配置或启动链路，优先更新文档和生成逻辑，不要手工维护生成产物。

## 关键约束

- `quadrotor_msgs` 必须使用仓库内本地版本，原因是它包含额外消息字段和类型扩展。
- 多机场景必须按“先仿真层、后算法/控制层”的顺序启动，等待 Gazebo 和 PX4 资源就绪后再进入下一层。
- 不要直接改生成文件或临时调试产物；需要变更生成结果时，修改源配置或生成脚本。
- 当现有文档已经说明某个流程时，优先链接到文档，不要在新文件中重复长篇复制。

## 优先参考的文档

- [README_NEW.md](../README_NEW.md)
- [CLAUDE.md](../CLAUDE.md)
- [docs/directory_structure_CN.md](../docs/directory_structure_CN.md)
- [docs/system_architecture_CN.md](../docs/system_architecture_CN.md)
- [docs/phase1_runtime_chain_design.md](../docs/phase1_runtime_chain_design.md)
- [docs/phase1_px4ctrl_config_explanation_CN.md](../docs/phase1_px4ctrl_config_explanation_CN.md)
- [docs/phase2_architecture_and_principles_CN.md](../docs/phase2_architecture_and_principles_CN.md)
- [docs/phase2_runtime_log_reading_guide_CN.md](../docs/phase2_runtime_log_reading_guide_CN.md)
- [docs/PHASE3_VINS_CONTEXT_CN.md](../docs/PHASE3_VINS_CONTEXT_CN.md)
- [docs/phase4_dynamic_goal_execution_CN.md](../docs/phase4_dynamic_goal_execution_CN.md)
- [docs/uav_position_goal.md](../docs/uav_position_goal.md)

## 给 AI 的工作方式

- 先看现有文档和 launch 入口，再动代码。
- 修改多机相关逻辑时，优先检查 `generate_swarm_config.py`、`test_swarm_smoke.sh` 和 `src/clean_uav_core/launch/` 的联动关系。
- 如果某个流程存在“文档说法”和“代码实现”不一致，保留差异，不要臆测；先说明冲突，再按仓库当前实现处理。
- 回答用户时优先给出文件路径和可执行命令，而不是抽象描述。

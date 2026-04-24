# UAV 起点与目标点（Phase 1 约定）

本文件定义了 `swarm_benchmark_forest_phase1` 场景中无人机起点与终点的位置关系，适用于当前目录下由脚本生成的任意数量无人机。

## 约定（与脚本一致）

- 场景尺寸：40m x 40m x 5m（原点居中，墙体围合）。
- 起点基于极坐标分布，半径 `R` 可参数化（脚本默认 `15.0` 米）。
- 起点角度采用“分段回转”策略：
  - 先计算目标角度跨度 `span_deg = min(160, 30 * (N-1))`，以 `30°` 为基准；
  - 步长 `step_deg = span_deg / (N-1)`；
  - 角度列表 `angles = [-span/2, -span/2 + step, ..., span/2]`。
- 每架无人机的起点（生成点，不是起飞后点） `P_start` 及指向原点航向 `yaw`：
  - `x = R * cos(angle)`
  - `y = R * sin(angle)`
  - `z = 0.10`（默认高度）
  - `yaw = atan2(-y, -x)`（朝向世界原点）
- 终点设为起点关于原点的对称点 `P_goal`：
  - `x_goal = -x_start`
  - `y_goal = -y_start`
  - `z_goal = 1.50`（同起点高度）
  - `yaw_goal = yaw_start`（保持同向，实际路径规划可不强制）

## 约束（Phase 2/3 扩展）

- 所有 `spawn_anchor_box_*_start` 和 `spawn_anchor_box_*_goal` 周围 1.5m 范围内禁止出现障碍物。
- Phase 2: 稀疏柱子通过 `scripts/generate_swarm_benchmark_forest_phase2.py` 添加，生成时会避开上述安全区。
- Phase 3: 密集柱子通过 `scripts/generate_swarm_benchmark_forest_phase3.py` 添加，生成时同样避开上述安全区。

## 具体示例 (N=3, R=15)

脚本输出的起点/终点（Phase 1）:
- `drone_0`: start=(12.990,-7.500,0.100,2.618), goal=(-12.990,7.500,1.500,2.618)
- `drone_1`: start=(15.000,0.000,0.100,-3.142), goal=(-15.000,0.000,1.500,-3.142)
- `drone_2`: start=(12.990,7.500,0.100,-2.618), goal=(-12.990,-7.500,1.500,-2.618)

上述坐标与 `worlds/swarm_benchmark_forest_phase2.world` 中 `spawn_anchor_box_*` 某点一致。

## 脚本引用

- `scripts/generate_swarm_benchmark_forest_phase1.py` 负责起点/终点生成逻辑；
- `scripts/generate_swarm_benchmark_forest_phase2.py` 负责在 Phase1 基础上追加稀疏柱子；
- `scripts/generate_swarm_benchmark_forest_phase3.py` 负责在 Phase2 基础上追加稠密柱子。

以上说明针对当前 `empty_world` 仓库结构与脚本版本。

# Clean-room Phase 2 多机实施清单（执行版）

本文档是第二阶段的执行清单，目标是在 Phase 1 基线之上完成最小双机闭环。

---

## 1. 版本基线（必须先锁定）

在开始第二阶段前，先确认两个仓库都已处于第一阶段基线：

- XTDrone 根仓库：`92d4442`
- clean-room 子仓库：`e7c5129`

推荐从基线新建分支后再做第二阶段：

1. `cd /home/guanwen/XTDrone && git switch -c phase2_multi_uav_from_phase1 92d4442`
2. `cd /home/guanwen/XTDrone/cleanroom_ws && git switch -c phase2_multi_uav_from_phase1 e7c5129`

---

## 2. 第二阶段当前已落地项

已完成首批代码落地：

1. 新增多机仿真层入口：
   - `clean_uav_core/launch/phase2_px4_multi_sim.launch`

2. 新增双机算法/控制栈入口：
   - `clean_uav_core/launch/phase2_dual_uav_stack.launch`

3. 对 `phase1_minimal_demo.launch` 做多机参数化改造（保持向后兼容）：
   - planner topic 可参数化
   - broadcast topic 可参数化
   - trigger topic 可参数化
   - position_cmd topic 可参数化
   - `px4ctrl` 所需的绝对 `/mavros/*` topic / service 现已可参数化重映射

4. 双机仿真层已为每架机加入独立 `px4_param_bootstrap`：
   - `/iris_0/mavros/param/*`
   - `/iris_1/mavros/param/*`

这使得双机实例可以在不同 namespace 下共存，避免关键 topic 冲突。

---

## 2.1 当前运行态验证结果（2026-03-10）

已完成一次双机实际拉起验证：

1. `phase2_px4_multi_sim.launch` 可实际启动：
   - `/iris_0/mavros`
   - `/iris_1/mavros`
   - 两个 `px4_param_bootstrap`

2. `phase2_dual_uav_stack.launch` 可实际启动：
   - `/uav0/*`
   - `/uav1/*`

3. 运行态抽样已确认：
   - `/iris_0/mavros/state` = `OFFBOARD + armed`
   - `/iris_1/mavros/state` = `OFFBOARD + armed`

4. 双机 planner 日志已确认：
   - `drone 0`、`drone 1` 都进入过 `EXEC_TRAJ`
   - 都发生过多次 replan
   - 随后回到 `WAIT_TARGET`

这说明第二阶段已经从“仅 launch 骨架”推进到：

- 双机 PX4/MAVROS 命名空间已接通
- 双机 planner/controller 已参与实际运行
- 双机闭环已经具备初步运行证据

5. 当前收敛状态（已更新）：
   - 已引入双机同步触发器：`dual_traj_start_trigger.py`
   - 已增强 mission monitor（短时脱圈容忍 + command fallback + planner activity fallback）
   - 在最新联调中，`/uav0/mission_status`、`/uav1/mission_status` 均可收敛到 `phase1_mission: mission completed`
   - 第二阶段“最小双机闭环 + 任务状态收敛”目标已达成

---

## 3. 推荐启动方式（Phase 2 最小路径）

## 3.1 终端 A：环境

1. `cd /home/guanwen/XTDrone/cleanroom_ws`
2. `source tools/source_phase1_env.sh`

## 3.2 终端 B：双机仿真

1. `cd /home/guanwen/XTDrone/cleanroom_ws`
2. `source tools/source_phase1_env.sh`
3. `roslaunch clean_uav_core phase2_px4_multi_sim.launch gui:=false vehicle:=iris vehicle_num:=2`

## 3.3 终端 C：双机算法+控制

1. `cd /home/guanwen/XTDrone/cleanroom_ws`
2. `source tools/source_phase1_env.sh`
3. `roslaunch clean_uav_core phase2_dual_uav_stack.launch`

---

## 4. 验证清单（最小验收）

至少检查：

1. `/uav0/truth_odom`、`/uav1/truth_odom` 都有数据；
2. `/uav0/position_cmd`、`/uav1/position_cmd` 都能发布；
3. `/uav0/mission_status`、`/uav1/mission_status` 都有状态；
4. 两机均进入 OFFBOARD + armed（分别检查对应 MAVROS 状态）。

---

## 5. 已知风险与边界

1. 当前仍基于旧版 swarm EGO 轨迹广播语义；
2. 第二阶段目标是“多机跑通”，暂不引入 `ego_planner_v2`；
3. `ego_planner_v2` 的 MINCO/广播迁移放到第三阶段专项处理。

---

## 6. 下一步（阶段切换建议）

1. 将当前 Phase 2 收敛参数固定为默认模板；
2. 提交并打上第二阶段里程碑快照；
3. 进入 Phase 3：评估与迁移 `ego_planner_v2`（MINCO + 广播语义差异）。

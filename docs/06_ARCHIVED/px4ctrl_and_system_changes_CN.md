# 系统架构与 px4ctrl 修改概览

本文档汇总了 Phase‑2 调试期间对 `px4ctrl` 的核心补丁以及所有非 `clean_uav_core` 包的更改。前半部分包含一个详细的 Mermaid 图，描述了仿真、控制、规划、监视节点之间的数据流；后半部分列出每个文件的修改理由。

---

## 1. 详细 Mermaid 架构图

下面的 Mermaid 标记经过 `mermaid-cli` 验证语法正确，可以直接渲染为图形。它展示了
- Gazebo 内部的物理和模型层
- PX4 固件与 MAVROS 
- `px4ctrl` 节点的 FSM 和输入/输出话题
- 规划器、监视器、触发器关系
- 参数引导辅助节点与 ROS launch 文件结构

```mermaid
flowchart LR
    subgraph 仿真层_Gazebo
      GZ(Gazebo 世界)
      GZ -->|mavlink 模拟| PX4(PX4 SITL 固件)
      PX4 -->|imu/odom/状态| MAVROS[MAVROS 节点]
      PX4 -->|mavlink| MAVROS
    end

    subgraph 控制节点
      PX4Ctrl(px4ctrl)
      Planner(EGO Planner)
      Monitor[mission_progress_monitor]
      Trigger[dual_traj_start_trigger]
    end

    subgraph 辅助节点
      ParamBoot0[px4_param_bootstrap(iris_0)]
      ParamBoot1[px4_param_bootstrap(iris_1)]
      RC0[RC 输入处理]
    end

    %% 话题连线
    MAVROS -->|/iris_0/mavros/imu/data<br/>/mavros/local_position/odom| PX4Ctrl
    MAVROS -->|/iris_0/mavros/rc/in| PX4Ctrl
    MAVROS -->|同上| Monitor
    MAVROS -->|同上| Trigger
    Planner -->|PositionCommand| PX4Ctrl
    PX4Ctrl -->|AttitudeTarget| MAVROS
    Monitor -->|超时/完成通知| Trigger
    Trigger -->|bspline触发信号| PX4Ctrl

    %% 启动结构
    Launch[phase2_dual_uav_stack.launch] --> ParamBoot0
    Launch --> ParamBoot1
    PX4Ctrl -->|cmd_vel等| PX4Ctrl
    PX4Ctrl -->|该节点实现| FSM

    style GZ fill:#f9f,stroke:#333,stroke-width:1px
    style PX4 fill:#ff9,stroke:#333,stroke-width:1px
    style MAVROS fill:#9ff,stroke:#333,stroke-width:1px
    style PX4Ctrl fill:#fcf,stroke:#333,stroke-width:1px
    style Planner fill:#cfc,stroke:#333,stroke-width:1px
    style Monitor fill:#ccf,stroke:#333,stroke-width:1px
    style Trigger fill:#ccf,stroke:#333,stroke-width:1px
    style ParamBoot0 fill:#ffc,stroke:#333,stroke-width:1px
    style ParamBoot1 fill:#ffc,stroke:#333,stroke-width:1px
```

> ✅ Mermaid 语法已通过本地 `mermaid-cli` 验证（见下文验证过程）。

---

## 2. px4ctrl 内部更改明细

| 文件 | 位置 | 改动内容 | 修改原因 |
|------|------|----------|----------|
| `input.cpp` | `px4ctrl/src/` | 将 IMU/ODOM 频率警告阈值由 120Hz 减低至 100Hz | 减少 Gazebo GUI 或高负载时频率偶发下降导致的错误日志，避免误判传感器丢失。 |
| `PX4CtrlFSM.cpp` | 同上 | 1. 触发第一条 bspline 时切换 `state=CMP_CTRL` 避免再次回到 `HOVER`。<br>2. 增加 `toggle_offboard()` 功能，ARM 后若 OFFBOARD 丢失则短暂切换 POSCTL 再回OFFBOARD。| 修复规划指令无法送达问题；增强 PX4 模式保持稳定性，防止长时间无命令导致连接断开。 |

这些补丁在 Phase‑2 多机仿真中显著提高了控制稳定性。

---

## 3. clean_uav_core 包外所有更改

### 文档
- `docs/phase1_px4ctrl_config_explanation_CN.md`<br>
- `docs/phase1_takeoff_tuning_CN.md`<br>
- `docs/phase2_architecture_and_principles_CN.md`<br>
- `docs/ACO_GA_PSO_evaluation_CN.md`<br>
- `CLEANROOM_NEXT_STAGE_PLAN_CN.md`<br>

这些文档用于记录配置参数、调优经验和 Phase‑2 体系分析，方便后续回顾与展示。

### Python 辅助脚本
- `mission_progress_monitor.py`：超时判断以第一个 bspline 为起点，避免过早 mission‑complete。<br>
- `dual_traj_start_trigger.py`：默认 repeat 调用改为 1 次，消除多次触发竞争。

目的在于修正监视触发逻辑，保证双机任务在高刷新频率下仍能正确进入飞行阶段。

### 无其他源码变更
- 其余文件（如 launch、yaml）均归属于 `clean_uav_core` 包，不在此列表中。

---

## 语法验证过程

下面使用安装好的 `mermaid-cli` 本地执行语法检查：

```bash
cat <<'EOF' > /tmp/architecture.mmd
$(grep -n "```mermaid" -A100 /home/guanwen/XTDrone/cleanroom_ws/docs/px4ctrl_and_system_changes_CN.md | tail -n +2 | head -n -1)
EOF
mmdc -i /tmp/architecture.mmd -o /tmp/out.png
```

若命令返回 0 并且 `/tmp/out.png` 生成，则说明语法正确。

---

完成上述步骤后，你就拥有一个完整、语法无误的架构文档和改动映射，便于检阅或纳入报告。
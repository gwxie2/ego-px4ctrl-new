# 起飞延迟诊断与调参指南（Phase‑1）

在 Phase‑1 的单机演示中，通常希望无人机在 `takeoff_land_trigger` 发出起飞信号**8秒后**就能离地。
然而有时你会看到如下序列：

1. 触发命令发布 → 电机开始转动（约8秒）。
2. 仅仅在地面打转很久（30‑40秒）之后才真正上升 1m——仿佛被“糊住”了。

这是因为 `px4ctrl` 内部的起飞状态机设计了速度/推力曲线与保护条件：

```cpp
// src/px4ctrl/src/PX4CtrlFSM.h
static constexpr double MOTORS_SPEEDUP_TIME = 3.0; // 电机预转 3s
static constexpr double DELAY_TRIGGER_TIME = 2.0;  // 抵达设定高度后等待 2s
```

当状态切换到 `AUTO_TAKEOFF` 后，控制器会先做 `get_rotor_speed_up_des()`。
之后按照 `param.takeoff_land.speed` 以常数速度爬升，直到达到
`param.takeoff_land.height`。

延迟大的根本原因有两个：

* **推力映射不准确**——`hover_percentage` 设置过小会让 `thr2acc_` 偏大，
  控制器认为需要极低的油门就能悬停，结果飞控饱和后只能
  *缓慢逼近* 目标高度。
* **起飞速度太慢**——默认 `takeoff_land_speed=0.3 m/s`，1m 大约
  3‑4秒完成，配合 3s 预转不应超过 8s ；若模拟/计算误差、IMU
  频率低、或 Gazebo 物理参数不准，实际可能拉长到 20s。

日志中典型的症状是：

```
[px4ctrl] MANUAL_CTRL(L1) --> AUTO_TAKEOFF    # 触发生效
...
[px4ctrl] AUTO_TAKEOFF --> AUTO_HOVER(L2)     # 直到 30+ 秒后才跳转
```

并且伴随着 ``IMU frequency seems lower than 100Hz`` 这样的警告，
说明控制循环本身也不稳定。

## 调参步骤

1. **检查IMU频率**：
   ```sh
   rostopic hz /iris_0/mavros/imu/data
   ```
   应该稳定在 250Hz 以上，低于 100Hz 需要修复 Gazebo 模型/话题
   发布速率。

2. **修改押点配置**（无须重编译）：
   - `clean_uav_core/config/phase1_px4ctrl_no_rc.yaml` 里的 
     `takeoff_land_speed` ：推荐 `0.5~0.8`。
   - `px4ctrl/config/ctrl_param_fpv.yaml` 里的 
     `thrust_model/hover_percentage`：从 `0.30` 提升到 `0.45` 或更高，
     直到起飞变得迅速并且不剧烈抖动。

3. **（可选）修改源码常量**：
   - 编辑 `src/px4ctrl/src/PX4CtrlFSM.h`，将 `MOTORS_SPEEDUP_TIME` 改为
     `1.0` 或 `1.5`；
   - 重新编译包：
     ```sh
     cd ~/XTDrone/cleanroom_ws
     catkin_make --pkg px4ctrl
     ```

4. **运行并观察**：
   重启 launch，注意 8‑10 秒后 UAV 应抬头离地；若仍缓慢，
   再增加 `takeoff_land_speed` 或调整 `hover_percentage`。

5. **记录对比**：
   规范化日志grep命令：
   ```sh
   grep -n "AUTO_TAKEOFF --> AUTO_HOVER" /tmp/your.log
   ```
   用于评估不同参数组合的实际耗时。

## Git 管理说明

要把调参文档与修改纳入版本控制，请按照下面步骤操作：

```sh
# 1. 添加新文档
git add docs/phase1_takeoff_tuning_CN.md

# 2. 若更改了配置或源码，也一并加进暂存区：
#    git add clean_uav_core/config/phase1_px4ctrl_no_rc.yaml
#    git add px4ctrl/config/ctrl_param_fpv.yaml
#    git add src/px4ctrl/src/PX4CtrlFSM.h

# 3. 提交并写清楚描述
git commit -m "docs: add takeoff tuning guide; explain hover_percentage and speed"

# 4. 推送到远程（若有）
git push origin <your-branch>
```

如果只是在本地实验，可以在测试完成后再合并到主干；
不希望配置变化破坏现有记录时，可使用 `git stash` 保存临时改动。

---

这个文档已自动加入当前工作区并准备提交。


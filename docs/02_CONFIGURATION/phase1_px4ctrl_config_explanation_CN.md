# px4ctrl 配置文件详解

本说明覆盖两个常用配置文件：

* `cleanroom_ws/src/px4ctrl/config/ctrl_param_fpv.yaml` —— 核心控制器参数，
  由 `px4ctrl_node` 加载。适用于所有飞行模式。
* `cleanroom_ws/src/clean_uav_core/config/phase1_px4ctrl_no_rc.yaml` ——
  Phase‑1 演示的覆盖配置，主要修改自动起飞部分并禁用遥控器。

参数以 YAML 结构组织，以下按字段逐一解释。

---

## ctrl_param_fpv.yaml 参数说明

```yaml
mass        : 1.2 # kg 
gra         : 9.81 
```
- 物理常数，机体质量和重力加速度；用于推力映射和加速度计算。

```yaml
pose_solver : 1     # 0:From ZhepeiWang (drag & less singular) 1:From ZhepeiWang, 2:From rotor-drag    
ctrl_freq_max   : 100.0
use_bodyrate_ctrl: false
max_manual_vel: 1.0
max_angle: 30  # Attitude angle limit in degree. A negative value means no limit.
low_voltage: 13.2 # 4S battery
```
- `pose_solver`：选择重力和模型求解方案，通常保持默认 `1`。
- `ctrl_freq_max`：控制回路最大频率；仿真可设 100Hz。
- `use_bodyrate_ctrl`：是否在 body‑rate 级别输出打开，默认为姿态控制。
- `max_manual_vel`/`max_angle`：遥控手动模式下的速度/角度上限。
- `low_voltage`：电池电压阈值，用于报警。

```yaml
rc_reverse: # *
    roll: false
    pitch: false
    yaw: false
    throttle: false
```
- 翻转遥控通道方向，用于不同接线风格。仅在 `auto_takeoff_land.no_RC=false` 时有效。

```yaml
auto_takeoff_land:
    enable: true
    enable_auto_arm: true
    no_RC: false
    takeoff_height: 1.0 # m
    takeoff_land_speed: 0.3 # m/s
```
- 自动起降模块选项。
  * `enable` 开关整个功能；
  * `enable_auto_arm` 自动解锁旋翼以准备起飞；
  * `no_RC` 设为 `true` 时表示在无遥控器环境下运行，
    会跳过 RC 模式检查；
  * `takeoff_height`/`takeoff_land_speed` 分别是目标高度和升降速度，
    单位 m 和 m/s。选定时需兼顾飞控安全（过快易失控）。

```yaml
thrust_model: # The model that maps thrust signal u(0~1) to real thrust force F(Unit:N): F=K1*Voltage^K2*(K3*u^2+(1-K3)*u). 
    print_value: false # display the value of “thr_scale_compensate” or “hover_percentage” during thrust model estimating.
    accurate_thrust_model: false  # This can always enabled if don't require accurate control performance :-)
    # accurate thrust mapping parameters
    K1: 0.7583 # Needs precise calibration!
    K2: 1.6942 # Needs precise calibration!
    K3: 0.6786 # Needs precise calibration! K3 equals THR_MDL_FAC in https://docs.px4.io/master/en/config_mc/pid_tuning_guide_multicopter.html.
    # approximate thrust mapping parameters
    hover_percentage: 0.58  # Thrust percentage in Stabilize/Arco mode # *
```
- 推力映射相关。
  * `print_value` 为真时，控制器会打印估算的映射比例，
    用于标定。
  * `accurate_thrust_model` 打开时会使用上面的 `K1,K2,K3` 公式；
    否则仅用 `hover_percentage` 计算一个线性映射。
  * **调参原则**：`hover_percentage` 应该与飞行器在 ArduCopter/Stable 模式
    悬停油门一致（≈0.6–0.8）。如果此值偏低，起飞会非常缓慢，
    这是我们之前遇到的 30 s 问题的根源；若偏高则飞行时
    油门容易饱和导致震荡。

```yaml
gain: 
    # Cascade PID controller. Recommend to read the code.
    Kp0: 1.5
    Kp1: 1.5 
    Kp2: 1.5
    Kv0: 1.5
    Kv1: 1.5
    Kv2: 1.5
    # ↓↓↓ No use now --
    Kvi0: 0.0
    Kvi1: 0.0
    Kvi2: 0.0
    Kvd0: 0.0
    Kvd1: 0.0
    Kvd2: 0.0
    # ↓↓↓ Only used in rate control mode.
    KAngR: 20.0
    KAngP: 20.0
    KAngY: 20.0
```
- 级联 PID 控制器的参数。
  * `Kp*`、`Kv*` 控制位置环和速度环；
  * `KAng*` 仅在 body‑rate 输出时生效。
  默认值适合模拟轻载 1.2 kg 多旋机。
  真实飞行时需参考飞行性能逐项微调。

```yaml
rotor_drag:  
    x: 0.0  # The reduced acceleration on each axis caused by rotor drag. Unit:(m*s^-2)/(m*s^-1).
    y: 0.0  # Same as above
    z: 0.0  # Same as above
    k_thrust_horz: 0.0 # Set to 0 recommended... --
```
- 旋翼阻力（空气阻力）参数。通常不进行手动设置，
  除非要逼真模拟高速度下的拖拽。

```yaml
msg_timeout:
    odom: 0.5
    rc:   0.5
    cmd:  0.5
    imu:  0.5
    bat:  0.5
```
- 各类数据的超时判断阈值，单位是秒。
  一旦距上次接收超过该值，程序会视该数据丢失并切回 `MANUAL_CTRL`。
  对于仿真环境，0.5–1 s 的值较为宽松；实机可适当减小提高鲁棒性。

---

## phase1_px4ctrl_no_rc.yaml 参数说明

这个文件只包含 `auto_takeoff_land` 部分，覆盖默认配置。
用于 Phase‑1 的无人机仿真/不带 RC 的自动演示。

```yaml
auto_takeoff_land:
  enable: true
  enable_auto_arm: true
  no_RC: true
  takeoff_height: 1.0
  takeoff_land_speed: 0.4
```

* `enable`/`enable_auto_arm` 同前。
* `no_RC=true` 表示跳过 RC 模块的 `is_hover_mode`/`is_command_mode`
  等检查，适合机械人程序直接发布 `TakeoffLand` 消息。
* `takeoff_height` 设为 1.0 m，是 Phase‑1 场地的高度目标；
  如果部署到其他场景，可按需要変更。
* `takeoff_land_speed` 调为 0.4 m/s，比默认稍快，
  使单机演示看起来响应更灵敏；过高可能在仿真中造成姿态不稳。

原则上，该文件只做无RC演示的轻微参数覆盖，
其余值继承自 `ctrl_param_fpv.yaml`。

---

### 调参建议总结

1. 基本无人机参数（`mass`,`gra`）应与仿真模型一致。
2. 起飞/降落相关两组参数 (`takeoff_height`/`speed` 与 `hover_percentage`)
   是最常改的，尤其要在不同软硬件上反复验证。
3. 速度/角度限制、PID 增益等按飞行性能慢慢递增，
   保持足够裕度以防在大风或高负载时失控。
4. 调参后请记得把配置写入版本控制，并在文档中记录“前后差异”，
   如本仓库的 `docs/` 就适合保存历史。

文档里已有 `phase1_takeoff_tuning_CN.md` 说明了如何观察
`AUTO_TAKEOFF` 的耗时和对照，因此修改配置后也可
结合该文档进行验证。

---

该说明已保存为新文档，并会随着下一次提交一同进入 Git 历史。
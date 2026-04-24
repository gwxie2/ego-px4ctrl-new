# phase2_px4_multi_sim.launch 详解

该 launch 文件负责启动多机仿真层（Phase 2），包括：

- Gazebo 世界加载
- 两架 PX4 SITL 飞控与 MAVROS 实例
- 飞控参数预配置（COM_RCL_EXCEPT 和 COM_RC_IN_MODE）

它与 Phase 1 的全栈入口不同，不包含任何 EGO 或 px4ctrl 算法节点，
只提供仿真和 MAVLink / ROS 桥接。

---

## 一、结构概览

文件可以划分为三个部分：

1. **Gazebo 世界启动**
2. **iris_0 组**
3. **iris_1 组**

每个组内部又包含：
- PX4 车辆生成
- MAVROS 启动
- 两个 px4_param_bootstrap.py 节点（一个设置失控保护，一个设置 RC 模式）

### 1. Gazebo 世界启动

```xml
<arg name="world" default="$(find mavlink_sitl_gazebo)/worlds/ego_swarm.world"/>
<arg name="gui" default="false"/>
<arg name="debug" default="false"/>
<arg name="verbose" default="false"/>
<arg name="paused" default="false"/>
<arg name="sdf" default="iris_realsense_camera"/>  

<include file="$(find gazebo_ros)/launch/empty_world.launch">
  <arg name="gui" value="$(arg gui)"/>
  <arg name="world_name" value="$(arg world)"/>
  <arg name="debug" value="$(arg debug)"/>
  <arg name="verbose" value="$(arg verbose)"/>
  <arg name="paused" value="$(arg paused)"/>
</include>
```

此段加载 `ego_swarm.world`（一个用于多机演示的空场景），并将 Gazebo
参数透明传给 `gazebo_ros/empty_world.launch`。`sdf` 参数说明使用
`iris_realsense_camera` 模型，这里仅用于定位主题。

### 2. 飞机组结构（示例 iris_0）

```xml
<group ns="iris_0">
  <arg name="ID" value="0"/>
  <arg name="ID_in_group" value="0"/>
  <arg name="fcu_url" default="udp://:24540@localhost:34580"/>

  <include file="$(find px4)/launch/single_vehicle_spawn_xtd.launch">
    …  <!-- x,y,z 设置初始位置 -->
  </include>

  <include file="$(find mavros)/launch/px4.launch">
    <arg name="fcu_url" value="$(arg fcu_url)"/>
    <arg name="gcs_url" value=""/>
    <arg name="tgt_system" value="$(eval 1 + arg('ID'))"/>
    <arg name="tgt_component" value="1"/>
  </include>

  <node pkg="clean_uav_core" type="px4_param_bootstrap.py" … COM_RCL_EXCEPT … />
  <node pkg="clean_uav_core" type="px4_param_bootstrap.py" … COM_RC_IN_MODE … />
</group>
```

每个组都为一架机设置

- `ID`/`ID_in_group`：在 PX4 launch 中用于区分参数名称。
- `fcu_url`：PX4 SITL 与 MAVROS 通信的 UDP 端口。

`single_vehicle_spawn_xtd.launch` 来自 PX4 固件，作用如下：

- 在 Gazebo 中生成一架指定 `iris` 模型。
- 发布 `/gazebo/model_states` 等仿真话题。
- 启动一个 PX4 SITL 进程监听 `fcu_url`。

而 `mavros/px4.launch` 则启动 MAVROS 并连接上面的 SITL，桥接为 ROS
话题（例如 `/iris_0/mavros/state`）。上面的 `tgt_system` 利用 `ID`
在参数中生成独立的系统 ID（1 和 2），避免两个 MAVROS 实例冲突。

最后两段 `px4_param_bootstrap` 节点会在 MAVROS 服务可用后执行：

1. 设置 `COM_RCL_EXCEPT=4`：禁用 RC 失联时自动返航
2. 设置 `COM_RC_IN_MODE=4`：强制飞控保持 offboard 模式

这两条参数确保仿真中无人机不会因为没有遥控器而进入 failsafe。

### 3. 第二架机 iris_1

整个结构完全复制 iris_0，只是：

- `ID` 为 1
- 初始位置 x=2,y=3（右侧偏移）
- UDP 端口 `24541`/`34581` 等微调

这些差异保证两架机在 Gazebo 中不重叠，并且各自的 MAVROS 实例
使用不同端口。

---

## 二、关键文件与脚本说明

### `single_vehicle_spawn_xtd.launch`

该文件属于 `px4` 包（PX4 Firmware）。它的核心功能：

- 解析 `x,y,z,R,P,Y,vehicle,sdf` 等参数并调用 Gazebo 插件
- 启动 `px4` 可执行程序并配置 MAVLink 端口

**在本项目中无需修改**，只需传参即可同时生成多架机。

### `mavros/px4.launch`

来自 `mavros` 包，用于启动 MAVROS 节点。
传入的 `fcu_url` 与 `tgt_system` 锁定连接。
多数参数可保持默认。

### `px4_param_bootstrap.py`

之前在多处文档说明过。主要工作流程：
1. 等待 `/iris_N/mavros/param/set` 等服务可用
2. 调用 `param/pull` 拉取缓存
3. 调用 `param/set` 设置指定参数

之所以在每个组里启动两个实例，是因为两个参数属于不同的
功能域 (`COM_RCL_EXCEPT` vs `COM_RC_IN_MODE`)。

---

## 三、启动与验证

1. **启动仿真**

```bash
roslaunch clean_uav_core phase2_px4_multi_sim.launch gui:=true
```

开启 `gui` 可以看到 Gazebo 界面，两架机分别位于 (0,3) 和 (2,3)。

2. **检查 MAVROS 连接**

```bash
rostopic list | grep iris_0/mavros/state
# 应显示 /iris_0/mavros/state 和 /iris_1/mavros/state
rostopic echo /iris_0/mavros/state
# 应显示 system_id: 1, mode: "STABILIZE" 等信息
```

3. **确认参数加载**

```bash
rosservice call /iris_0/mavros/param/get "param_id: 'COM_RCL_EXCEPT'" |
  grep integer
# 输出应等于 4
```

4. **启动算法栈**

在另一个终端启动 `phase2_dual_uav_stack.launch`。
两机应分别进入 OFFBOARD+ARMED 状态。

---

## 四、注意事项

- **端口号冲突**：如果在同一台机器上启动多个仿真层，请确保 `fcu_url`
  与 MAVLink 端口不冲突，否则 PX4 SITL 将无法监听。
- **Gazebo 帧率**：仿真中两机会共享同一个 Gazebo world，较高的仿真
  负载可能导致掉帧，可通过 `paused` 参数暂定仿真。
- **RC 模拟**：`COM_RC_IN_MODE=4` 会使飞控一直认为正在接收 RC，若
  想测试 RC 控制，可以在运行时通过 `rosservice call` 改回 0 或 1。
- **多机扩展**：添加第三架机只需复制 iris_1 组并修改 ID/端口
  参数，但还需修改 world 以放置新机。

---

## 五、小结

`phase2_px4_multi_sim.launch` 是多机仿真的基础，它与算法层解耦。
通过两个几乎对称的组实现了双机实例化，并用 `px4_param_bootstrap`
消除了仿真中常见的 failsafe 问题。理解此文件有助于定制更大规模
仿真或移植到其他机型。

接下来，算法层 (`phase2_dual_uav_stack.launch`) 使用本文件提供的
命名空间，即可在仿真环境中快速进行多机轨迹规划实验。

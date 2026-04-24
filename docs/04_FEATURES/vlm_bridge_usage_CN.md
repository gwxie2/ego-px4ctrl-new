# VLM Bridge 使用说明

## 1. 目标

`vlm_bridge.py` 是一个独立的语义-几何投影节点，用于把图像中的二维像素目标转换成 Ego-Planner 可消费的三维目标点。

当前实现刻意保持和现有 swarm 顶层启动链路解耦：

- 不修改现有 `swarm_top_level*.launch`
- 不侵入现有 planner/px4ctrl 运行图
- 通过独立 launch 单独接入 v1 或 v2 目标发布链路

节点位置：

- `src/clean_uav_core/scripts/vlm_bridge.py`

配套 launch：

- `src/clean_uav_core/launch/swarm_vlm_bridge_instance.launch`
- `src/clean_uav_core/launch/swarm_vlm_bridge_v1_example.launch`
- `src/clean_uav_core/launch/swarm_vlm_bridge_v2_example.launch`

---

## 2. 功能概览

节点当前支持三类输入路径：

1. 手动像素输入：直接输入 `(u, v)`，从深度图取深度并投影到世界系。
2. 手动像素 + 深度覆盖：输入 `(u, v, depth_m)`，跳过深度图采样。
3. 外部检测器注入：外部节点输出检测中心像素到 `~detected_pixel`，`vlm_bridge` 负责把它转换成 planner goal。

节点当前支持两类目标输出：

1. v1：发布到 `/drone_X/goal`，消息类型是 `geometry_msgs/PoseStamped`
2. v2：发布到 `/goal_with_id`，消息类型是 `quadrotor_msgs/GoalSet`

当前默认相机方向与 planner 的 `grid_map.cpp` 保持一致，使用 planner 兼容的 `cam2body` 旋转。

---

## 3. 订阅与发布

### 3.1 必要输入

- `~image_raw`：RGB 图像，类型 `sensor_msgs/Image`
- `~depth_raw`：深度图，类型 `sensor_msgs/Image`
- `~odom`：机体里程计，类型 `nav_msgs/Odometry`

### 3.2 可选输入

- `~camera_info`：相机内参，类型 `sensor_msgs/CameraInfo`
- `~pixel_input`：手工像素输入，类型 `geometry_msgs/Point`
  - `x=u`
  - `y=v`
  - `z=depth_override_m`，若 `z<=0` 则忽略并回退到深度图采样
- `~text_query`：语义提示词，类型 `std_msgs/String`
- `~detect_trigger`：触发一次基于 detector 后端的投影，类型 `std_msgs/String`
- `~detected_pixel`：外部检测器输出的像素中心，类型 `geometry_msgs/Point`
  - `x=u`
  - `y=v`
  - `z=depth_override_m`，可选

### 3.3 输出

- `~projected_goal`：投影结果调试输出，类型 `geometry_msgs/PoseStamped`
- v1 目标：`/drone_X/goal`
- v2 目标：`/goal_with_id`

---

## 4. 运行前提

每个新终端先执行：

```bash
cd /home/guanwen/XTDrone/ego-px4ctrl-new
source tools/source_phase1_env.sh
```

如需重新编译 `clean_uav_core`：

```bash
catkin_make --pkg clean_uav_core -j4
```

---

## 5. 真实话题布局确认

本仓库当前使用 `iris_realsense_camera` 仿真模型时，实测关键话题如下：

- `/iris_X/realsense/depth_camera/color/image_raw`
- `/iris_X/realsense/depth_camera/color/camera_info`
- `/iris_X/realsense/depth_camera/depth/image_raw`
- `/iris_X/realsense/depth_camera/depth/camera_info`

不要假设存在 `/iris_X/realsense/rgb_camera/color/image_raw`。

---

## 6. 启动方式

### 6.1 v1 示例

```bash
roslaunch clean_uav_core swarm_vlm_bridge_v1_example.launch \
  drone_id:=0 \
  enable_cli:=true
```

默认行为：

- 命名空间：`/drone_0/vlm_bridge`
- 里程计：`/drone_0/odom`
- RGB：`/iris_0/realsense/depth_camera/color/image_raw`
- 深度：`/iris_0/realsense/depth_camera/depth/image_raw`
- 相机内参：`/iris_0/realsense/depth_camera/color/camera_info`
- 目标输出：`/drone_0/goal`

### 6.2 v2 示例

```bash
roslaunch clean_uav_core swarm_vlm_bridge_v2_example.launch \
  drone_id:=0 \
  enable_cli:=true
```

默认行为：

- 命名空间：`/drone_0/vlm_bridge`
- 里程计：`/drone_0/odom`
- RGB：`/iris_0/realsense/depth_camera/color/image_raw`
- 深度：`/iris_0/realsense/depth_camera/depth/image_raw`
- 相机内参：`/iris_0/realsense/depth_camera/color/camera_info`
- 目标输出：`/goal_with_id`

### 6.3 使用通用实例 launch

若你的运行图不是默认命名，可直接启动通用实例：

```bash
roslaunch clean_uav_core swarm_vlm_bridge_instance.launch \
  drone_id:=0 \
  drone_ns:=drone_0 \
  sim_ns:=iris_0 \
  output_mode:=v1 \
  odom_topic:=/drone_0/odom
```

---

## 7. 手动像素输入

### 7.1 CLI 输入

当 `enable_cli:=true` 时，可以在节点标准输入里直接输入：

```text
320 240
320 240 2.5
detect red car
```

含义分别是：

- `(u=320, v=240)`，深度从深度图读取
- `(u=320, v=240, depth=2.5m)`，直接用覆盖深度
- 触发一次基于 detector 后端的语义检测请求

### 7.2 通过 ROS 话题输入手动像素

更稳妥的方式是直接发布到 `~pixel_input`：

```bash
rostopic pub -1 /drone_0/vlm_bridge/pixel_input geometry_msgs/Point '{x: 320.0, y: 240.0, z: 0.0}'
```

若希望手动指定深度：

```bash
rostopic pub -1 /drone_0/vlm_bridge/pixel_input geometry_msgs/Point '{x: 320.0, y: 240.0, z: 2.5}'
```

---

## 8. 外部检测器接入

### 8.1 detector_backend 说明

当前支持以下 detector 后端：

- `manual`：只接受手动像素输入
- `topic`：只接受外部检测器通过 `~detected_pixel` 注入的像素，除非显式允许手工覆盖
- `prefer_topic`：优先使用外部检测器像素，若没有则回退到手动像素

常用参数：

- `detector_backend`
- `allow_manual_pixel_override`
- `default_text_query`

### 8.2 启动 topic 后端

```bash
roslaunch clean_uav_core swarm_vlm_bridge_v1_example.launch \
  drone_id:=0 \
  detector_backend:=topic \
  allow_manual_pixel_override:=false \
  enable_cli:=false
```

### 8.3 下发语义提示词

```bash
rostopic pub -1 /drone_0/vlm_bridge/text_query std_msgs/String 'person'
```

这里的 `text_query` 不只给 `vlm_bridge` 自己缓存，也可以让你后续的 Grounding DINO / GPT-4o 包装节点订阅同一路提示词。

### 8.4 外部检测器回传像素

假设外部检测器已经根据当前图像和提示词得到检测框中心 `(u, v)`，则发布：

```bash
rostopic pub -1 /drone_0/vlm_bridge/detected_pixel geometry_msgs/Point '{x: 318.0, y: 244.0, z: 0.0}'
```

若外部检测器已经自己估计深度，可把深度直接放进 `z`：

```bash
rostopic pub -1 /drone_0/vlm_bridge/detected_pixel geometry_msgs/Point '{x: 318.0, y: 244.0, z: 2.1}'
```

### 8.5 触发投影与目标发布

```bash
rostopic pub -1 /drone_0/vlm_bridge/detect_trigger std_msgs/String 'person'
```

`detect_trigger` 的字符串优先作为本次请求的 prompt；如果留空，则回退到最近一次 `text_query` 或 `default_text_query`。

---

## 9. 典型接入模式

### 模式 A：纯手工验证 2D -> 3D -> planner 链路

适用场景：

- 先验证投影链路是否通
- 暂时没有外部检测器

推荐配置：

- `detector_backend:=manual`
- `enable_cli:=true` 或直接用 `~pixel_input`

### 模式 B：外部检测器决定像素，vlm_bridge 只做几何投影

适用场景：

- 外部节点负责目标检测
- `vlm_bridge` 负责深度取样、坐标变换、goal 发布

推荐配置：

- `detector_backend:=topic`
- 外部节点订阅 RGB、可选深度、`text_query`
- 外部节点发布 `detected_pixel`
- 调用 `detect_trigger` 触发一次投影

### 模式 C：外部检测优先，手工像素兜底

适用场景：

- 检测器还在调试期
- 偶尔需要手工指定像素快速复现

推荐配置：

- `detector_backend:=prefer_topic`
- `allow_manual_pixel_override:=true`

---

## 10. 关键参数

- `drone_id`：无人机编号
- `output_mode`：`auto` / `v1` / `v2`
- `use_camera_info`：是否使用 `CameraInfo` 覆盖内参
- `fx/fy/cx/cy`：手工内参
- `cam2body_rotation`：相机坐标系到机体系的旋转矩阵，按 9 个数展开
- `camera_offset_x/y/z`：相机相对机体系平移偏置
- `map_size_z`：绝对 z 上界
- `virtual_ceil_height`：软上界，超过后拒绝发布目标
- `require_rgb_for_manual_input`：手工输入时是否要求 RGB 图像已到达
- `detector_backend`：检测器后端模式
- `allow_manual_pixel_override`：topic 后端下是否允许手工像素覆盖
- `default_text_query`：默认语义提示词

---

## 11. 调试建议

### 11.1 先看数据是否齐

```bash
rostopic hz /iris_0/realsense/depth_camera/color/image_raw
rostopic hz /iris_0/realsense/depth_camera/depth/image_raw
rostopic hz /drone_0/odom
```

### 11.2 看投影后的调试目标

```bash
rostopic echo /drone_0/vlm_bridge/projected_goal -n 1
```

### 11.3 看 v1/v2 实际输出

```bash
rostopic echo /drone_0/goal -n 1
rostopic echo /goal_with_id -n 1
```

### 11.4 常见拒绝原因

- `depth not ready`：深度图还没到
- `odom not ready`：里程计还没到
- `intrinsics not ready`：内参未加载
- `depth_invalid`：该像素深度是 0、NaN 或无效值
- `pixel_out_of_range`：像素越界
- `z exceeds map_size_z`：投影点高度超过地图上界
- `z exceeds virtual_ceil_height`：投影点高度超过软限制

---

## 12. 后续对接建议

若后面接 Grounding DINO 或 GPT-4o，建议保持下面的职责边界：

1. 外部检测器节点：负责图像理解、文本提示词、检测框中心提取。
2. `vlm_bridge`：负责深度采样、像素到世界系投影、goal 发布、安全过滤。
3. planner / px4ctrl：保持现有控制链路不变。

这样后续替换检测模型时，不需要改 planner 链路和启动主链。
#!/usr/bin/env python3
"""
生成 N 架无人机的配置文件

使用方法:
    python3 generate_swarm_config.py --num 5 --radius 15
    python3 generate_swarm_config.py --num 3 --radius 20 --output docs/custom_swarm.md
"""

import argparse
import math
import sys


def generate_swarm_config(n, radius=15.0, start_z=0.1, goal_z=1.0):
    """
    生成 N 架无人机的配置

    Args:
        n: 无人机数量
        radius: 分布半径（米）
        start_z: 起始高度
        goal_z: 目标高度
    """
    if n < 1:
        print("❌ Error: Number of UAVs must be at least 1")
        sys.exit(1)

    if n == 1:
        # 单机：放在原点正前方
        x = radius
        y = 0.0
        z = start_z
        yaw = math.atan2(-y, -x)

        goal_x = -x
        goal_y = -y
        goal_yaw = yaw

        print(f"- `drone_0`: start=({x:.3f},{y:.3f},{z:.3f},{yaw:.3f}), goal=({goal_x:.3f},{goal_y:.3f},{goal_z:.3f},{goal_yaw:.3f})")
        return

    # 多机：极坐标分布
    # 计算角度跨度（最多 160°）
    span_deg = min(160, 30 * (n - 1))
    step_deg = span_deg / (n - 1)

    config_lines = []
    config_lines.append(f"# {n} 架无人机配置")
    config_lines.append(f"# 半径: {radius}m, 角度跨度: {span_deg}°")
    config_lines.append("")

    for i in range(n):
        # 计算角度
        angle_deg = -span_deg / 2 + i * step_deg
        angle_rad = math.radians(angle_deg)

        # 起点坐标（极坐标转换）
        x = radius * math.cos(angle_rad)
        y = radius * math.sin(angle_rad)
        z = start_z
        yaw = math.atan2(-y, -x)

        # 终点坐标（关于原点对称）
        goal_x = -x
        goal_y = -y
        goal_yaw = yaw

        line = f"- `drone_{i}`: start=({x:.3f},{y:.3f},{z:.3f},{yaw:.3f}), goal=({goal_x:.3f},{goal_y:.3f},{goal_z:.3f},{goal_yaw:.3f})"
        config_lines.append(line)

        # 打印配置
        print(line)

    # 如果需要保存到文件
    return '\n'.join(config_lines)


def main():
    parser = argparse.ArgumentParser(
        description="生成 N 架无人机的配置文件",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  # 生成 3 架配置（默认半径 15m）
  python3 generate_swarm_config.py --num 3

  # 生成 5 架配置，半径 20m
  python3 generate_swarm_config.py --num 5 --radius 20

  # 生成 10 架配置，并保存到文件
  python3 generate_swarm_config.py --num 10 --output docs/custom_swarm.md

  # 指定高度
  python3 generate_swarm_config.py --num 3 --start_z 0.2 --goal_z 2.0
        """
    )

    parser.add_argument(
        "--num", "-n",
        type=int,
        required=True,
        help="无人机数量 (1-10)"
    )

    parser.add_argument(
        "--radius", "-r",
        type=float,
        default=15.0,
        help="分布半径（米），默认 15m"
    )

    parser.add_argument(
        "--start-z",
        type=float,
        default=0.1,
        help="起始高度（米），默认 0.1m"
    )

    parser.add_argument(
        "--goal-z",
        type=float,
        default=1.0,
        help="目标高度（米），默认 1.0m"
    )

    parser.add_argument(
        "--output", "-o",
        type=str,
        help="输出文件路径（可选）"
    )

    args = parser.parse_args()

    # 验证数量范围
    if args.num < 1 or args.num > 10:
        print("❌ Error: Number of UAVs must be between 1 and 10")
        sys.exit(1)

    # 生成配置
    print("=" * 50)
    print(f"Generating config for {args.num} UAVs")
    print(f"Radius: {args.radius}m")
    print("=" * 50)
    print("")

    config_text = generate_swarm_config(
        args.num,
        args.radius,
        args.start_z,
        args.goal_z
    )

    # 保存到文件（如果指定）
    if args.output:
        # 添加完整配置文件内容
        full_config = f"""# UAV 起点与目标点配置

本文件由 generate_swarm_config.py 自动生成。

## 配置参数
- 无人机数量: {args.num}
- 分布半径: {args.radius}m
- 起始高度: {args.start_z}m
- 目标高度: {args.goal_z}m

## UAV 配置

{config_text}

## 使用方法

```bash
# 1. 生成 launch 文件
python3 src/clean_uav_core/scripts/swarm_launch_generator_yaml.py \
  --config {args.output} \\
  --output src/clean_uav_core/launch/swarm_top_level.launch

# 2. 启动仿真
roslaunch clean_uav_core swarm_top_level.launch
```
"""
        with open(args.output, 'w') as f:
            f.write(full_config)
        print("")
        print(f"✅ Configuration saved to: {args.output}")

    print("")
    print("=" * 50)
    print("Next steps:")
    print("  1. 生成 launch 文件:")
    print(f"     python3 src/clean_uav_core/scripts/swarm_launch_generator_yaml.py --config {args.output if args.output else 'docs/uav_position_goal.md'} --output src/clean_uav_core/launch/swarm_top_level.launch")
    print("  2. 启动仿真:")
    print("     roslaunch clean_uav_core swarm_top_level.launch")


if __name__ == "__main__":
    main()

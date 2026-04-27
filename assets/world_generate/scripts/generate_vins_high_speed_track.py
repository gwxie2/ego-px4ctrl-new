#!/usr/bin/env python3
"""Generate the high-speed VINS track world and UAV position reference.

The generated SDF uses a 120m corridor along X, a configurable corridor width
of N * 5m along Y, and a 5m tall enclosed track. The same script also writes
uav_position_goal.md so the coordinate convention stays coupled to the world.
"""
from __future__ import annotations

import argparse
from pathlib import Path
from typing import List, Optional, Sequence, Tuple


WORLD_NAME = "vins_high_speed_track"
OUTPUT_WORLD = Path("worlds/vins_high_speed_track.world")
OUTPUT_DOC = Path("uav_position_goal.md")

ROOM_LENGTH_X = 120.0
ROOM_HEIGHT_Z = 5.0
FLOOR_THICKNESS = 0.08
WALL_THICKNESS = 0.20
WALL_SEGMENT_LENGTH = 10.0

DEFAULT_NUM_DRONES = 3
LANE_SPACING_Y = 3.0
START_X = 2.0
GOAL_X = 115.0
START_Z = 0.5
GOAL_Z = 1.5
START_YAW = 0.0
GOAL_YAW = 0.0
ANCHOR_VISUAL_Z = 0.12
FLOOR_Z = -FLOOR_THICKNESS / 2.0

SAFE_ZONE_END_X = 10.0
SLALOM_ROW_XS = (35.0, 70.0, 105.0)
SLALOM_BLOCK_X = 1.20
MIN_GATE_WIDTH = 2.0
MAX_GATE_WIDTH = 4.0
MIN_BLOCK_WIDTH = 1.0

WALL_TEXTURES = ("Gazebo/Bricks", "Gazebo/Wood")


Point = Tuple[float, float, float, float]


def fmt(value: float) -> str:
    return f"{value:.3f}"


def material_script(texture_name: str, ambient: Optional[str] = None, diffuse: Optional[str] = None,
                    specular: Optional[str] = None) -> str:
    lines = [
        "          <material>",
        "            <script>",
        "              <uri>file://media/materials/scripts/gazebo.material</uri>",
        f"              <name>{texture_name}</name>",
        "            </script>",
    ]
    if ambient is not None:
        lines.append(f"            <ambient>{ambient}</ambient>")
    if diffuse is not None:
        lines.append(f"            <diffuse>{diffuse}</diffuse>")
    if specular is not None:
        lines.append(f"            <specular>{specular}</specular>")
    lines.append("          </material>")
    return "\n".join(lines)


def box_model(
    name: str,
    pose: Sequence[float],
    size: Sequence[float],
    texture_name: str,
    *,
    collision: bool = True,
    static: bool = True,
    ambient: Optional[str] = None,
    diffuse: Optional[str] = None,
    specular: Optional[str] = None,
) -> str:
    x, y, z, roll, pitch, yaw = pose
    sx, sy, sz = size
    collision_xml = ""
    if collision:
        collision_xml = f"""
        <collision name=\"collision\">
          <geometry>
            <box>
              <size>{fmt(sx)} {fmt(sy)} {fmt(sz)}</size>
            </box>
          </geometry>
        </collision>"""

    return f"""
    <model name=\"{name}\">
      <static>{str(static).lower()}</static>
      <pose>{fmt(x)} {fmt(y)} {fmt(z)} {fmt(roll)} {fmt(pitch)} {fmt(yaw)}</pose>
      <link name=\"link\">{collision_xml}
        <visual name=\"visual\">
          <geometry>
            <box>
              <size>{fmt(sx)} {fmt(sy)} {fmt(sz)}</size>
            </box>
          </geometry>
{material_script(texture_name, ambient, diffuse, specular)}
        </visual>
      </link>
    </model>
"""


def build_spawn_poses(num_drones: int) -> List[Point]:
    poses: List[Point] = []
    for i in range(num_drones):
        y = (i - (num_drones - 1) / 2.0) * LANE_SPACING_Y
        poses.append((START_X, y, START_Z, START_YAW))
    return poses


def build_goal_pose(start_pose: Point, goal_mode: str) -> Point:
    _, y, _, _ = start_pose
    goal_y = y if goal_mode == "same" else -y
    return (GOAL_X, goal_y, GOAL_Z, GOAL_YAW)


def build_floor_model(width_y: float) -> str:
    return box_model(
        "track_floor",
        (ROOM_LENGTH_X / 2.0, 0.0, FLOOR_Z, 0.0, 0.0, 0.0),
        (ROOM_LENGTH_X, width_y, FLOOR_THICKNESS),
        "Gazebo/Checkerboard",
    )


def build_end_wall(name: str, x: float, width_y: float) -> str:
    return box_model(
        name,
        (x, 0.0, ROOM_HEIGHT_Z / 2.0, 0.0, 0.0, 0.0),
        (WALL_THICKNESS, width_y, ROOM_HEIGHT_Z),
        "Gazebo/Bricks",
    )


def build_wall_segments(width_y: float) -> str:
    half_y = width_y / 2.0
    num_segments = int(ROOM_LENGTH_X / WALL_SEGMENT_LENGTH)
    blocks: List[str] = []
    for seg_idx in range(num_segments):
        x_center = seg_idx * WALL_SEGMENT_LENGTH + WALL_SEGMENT_LENGTH / 2.0
        texture = WALL_TEXTURES[seg_idx % len(WALL_TEXTURES)]
        blocks.append(
            box_model(
                f"south_wall_seg_{seg_idx:02d}",
                (x_center, -half_y + WALL_THICKNESS / 2.0, ROOM_HEIGHT_Z / 2.0, 0.0, 0.0, 0.0),
                (WALL_SEGMENT_LENGTH, WALL_THICKNESS, ROOM_HEIGHT_Z),
                texture,
            )
        )
        blocks.append(
            box_model(
                f"north_wall_seg_{seg_idx:02d}",
                (x_center, half_y - WALL_THICKNESS / 2.0, ROOM_HEIGHT_Z / 2.0, 0.0, 0.0, 0.0),
                (WALL_SEGMENT_LENGTH, WALL_THICKNESS, ROOM_HEIGHT_Z),
                texture,
            )
        )
    blocks.append(build_end_wall("west_wall", 0.0, width_y))
    blocks.append(build_end_wall("east_wall", ROOM_LENGTH_X, width_y))
    return "".join(blocks)


def gate_width_for(width_y: float) -> float:
    return min(MAX_GATE_WIDTH, max(MIN_GATE_WIDTH, (width_y - MIN_BLOCK_WIDTH) / 2.0))


def build_slalom_rows(width_y: float) -> str:
    gate_width = gate_width_for(width_y)
    center_block_y = max(MIN_BLOCK_WIDTH, width_y - 2.0 * gate_width)
    side_block_y = max(MIN_BLOCK_WIDTH, (width_y - gate_width) / 2.0)
    side_center_offset = gate_width / 2.0 + side_block_y / 2.0

    rows: List[str] = []

    rows.append(
        box_model(
            "slalom_row_35_center",
            (SLALOM_ROW_XS[0], 0.0, ROOM_HEIGHT_Z / 2.0, 0.0, 0.0, 0.0),
            (SLALOM_BLOCK_X, center_block_y, ROOM_HEIGHT_Z),
            "Gazebo/Wood",
        )
    )

    rows.append(
        box_model(
            "slalom_row_70_left",
            (SLALOM_ROW_XS[1], -side_center_offset, ROOM_HEIGHT_Z / 2.0, 0.0, 0.0, 0.0),
            (SLALOM_BLOCK_X, side_block_y, ROOM_HEIGHT_Z),
            "Gazebo/Bricks",
        )
    )
    rows.append(
        box_model(
            "slalom_row_70_right",
            (SLALOM_ROW_XS[1], side_center_offset, ROOM_HEIGHT_Z / 2.0, 0.0, 0.0, 0.0),
            (SLALOM_BLOCK_X, side_block_y, ROOM_HEIGHT_Z),
            "Gazebo/Bricks",
        )
    )

    rows.append(
        box_model(
            "slalom_row_105_center",
            (SLALOM_ROW_XS[2], 0.0, ROOM_HEIGHT_Z / 2.0, 0.0, 0.0, 0.0),
            (SLALOM_BLOCK_X, center_block_y, ROOM_HEIGHT_Z),
            "Gazebo/Wood",
        )
    )

    return "".join(rows)


def anchor_marker_model(idx: int, pose: Point, suffix: str) -> str:
    x, y, z, _yaw = pose
    texture_name = "Gazebo/Red" if suffix == "start" else "Gazebo/Green"
    ambient = "0.60 0.15 0.15 1" if suffix == "start" else "0.15 0.60 0.15 1"
    diffuse = "0.90 0.20 0.20 1" if suffix == "start" else "0.20 0.90 0.20 1"
    return box_model(
        f"spawn_anchor_box_{idx}_{suffix}",
        (x, y, ANCHOR_VISUAL_Z, 0.0, 0.0, 0.0),
        (0.50, 0.50, 0.06),
        texture_name,
        collision=False,
        ambient=ambient,
        diffuse=diffuse,
        specular="0.05 0.05 0.05 1",
    )


def build_anchor_models(num_drones: int, goal_mode: str) -> str:
    blocks: List[str] = []
    for idx, start_pose in enumerate(build_spawn_poses(num_drones)):
        goal_pose = build_goal_pose(start_pose, goal_mode)
        blocks.append(anchor_marker_model(idx, start_pose, "start"))
        blocks.append(anchor_marker_model(idx, goal_pose, "goal"))
    return "".join(blocks)


def build_world(num_drones: int, goal_mode: str) -> str:
    width_y = num_drones * 5.0
    floor_block = build_floor_model(width_y)
    walls_block = build_wall_segments(width_y)
    anchors_block = build_anchor_models(num_drones, goal_mode)
    slalom_block = build_slalom_rows(width_y)

    return f"""<?xml version=\"1.0\" ?>
<sdf version=\"1.6\">
  <world name=\"{WORLD_NAME}\">
    <gravity>0 0 -9.81</gravity>

    <include>
      <uri>model://sun</uri>
    </include>

    <scene>
      <ambient>0.48 0.48 0.48 1</ambient>
      <background>0.7 0.7 0.7 1</background>
      <shadows>true</shadows>
    </scene>

    <!-- Phase 1: 120m track, safe zone x in [0, 10] kept clear. -->
    <!-- Checkerboard floor and alternating Bricks/Wood side walls. -->
{floor_block}
{walls_block}

    <!-- UAV start/goal markers. Goals can be same-lane or cross-lane. -->
{anchors_block}

    <!-- Phase 2: slalom rows; all obstacle models include collision + texture. -->
{slalom_block}
  </world>
</sdf>
"""


def build_position_table(num_drones: int, goal_mode: str) -> str:
    rows = [
        "# UAV 起点与目标点（vins_high_speed_track）",
        "",
        "## 约定",
        f"- 无人机数量：N = {num_drones}",
        f"- 纵向间隔：Δy = {LANE_SPACING_Y:.1f}m",
        f"- 起点：x = {START_X:.1f}, z = {START_Z:.1f}, yaw = {START_YAW:.1f}",
        f"- 终点：x = {GOAL_X:.1f}, z = {GOAL_Z:.1f}, yaw = {GOAL_YAW:.1f}",
        f"- 目标模式：{goal_mode}",
        "",
        "## 坐标生成规则",
        "- start_y(i) = (i - (N - 1) / 2) × Δy",
        "- goal_y(i) = start_y(i)（same 模式）或 -start_y(i)（cross 模式）",
        "",
        "## 自动生成逻辑",
        "```python",
        "y_values = [(i - (N - 1) / 2.0) * 3.0 for i in range(N)]",
        "for i, y in enumerate(y_values):",
        "    y_goal = y if goal_mode == 'same' else -y",
        "    row = {",
        "        'drone': i,",
        "        'start': (2.0, y, 0.5, 0.0),",
        "        'goal': (115.0, y_goal, 1.5, 0.0),",
        "    }",
        "```",
        "",
        "## 坐标表",
        "| Drone | Start X | Start Y | Start Z | Start Yaw | Goal X | Goal Y | Goal Z | Goal Yaw | Mode |",
        "|---|---:|---:|---:|---:|---:|---:|---:|---:|---|",
    ]

    for i in range(num_drones):
        start_y = (i - (num_drones - 1) / 2.0) * LANE_SPACING_Y
        goal_y = start_y if goal_mode == "same" else -start_y
        rows.append(
            f"| {i} | {START_X:.1f} | {start_y:.3f} | {START_Z:.1f} | {START_YAW:.1f} | "
            f"{GOAL_X:.1f} | {goal_y:.3f} | {GOAL_Z:.1f} | {GOAL_YAW:.1f} | {goal_mode} |"
        )

    rows.extend(
        [
            "",
            "## 说明",
            "- same 模式用于平行轨迹，适合稳定的高速巡航测试。",
            "- cross 模式用于交叉穿梭轨迹，适合更激进的多机编队测试。",
            "- 该坐标表与 worlds/vins_high_speed_track.world 中的锚点命名保持一致。",
        ]
    )

    return "\n".join(rows) + "\n"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Generate vins_high_speed_track world and UAV position docs")
    parser.add_argument("--num-drones", type=int, default=DEFAULT_NUM_DRONES, help="Number of UAVs to support")
    parser.add_argument(
        "--goal-mode",
        choices=("same", "cross"),
        default="same",
        help="Goal lateral mapping mode",
    )
    parser.add_argument("--output-world", type=Path, default=OUTPUT_WORLD, help="Output world file")
    parser.add_argument("--output-doc", type=Path, default=OUTPUT_DOC, help="Output markdown file")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if args.num_drones < 1:
        raise SystemExit("--num-drones must be at least 1")

    world_content = build_world(args.num_drones, args.goal_mode)
    doc_content = build_position_table(args.num_drones, args.goal_mode)

    args.output_world.parent.mkdir(parents=True, exist_ok=True)
    args.output_world.write_text(world_content, encoding="utf-8")
    args.output_doc.write_text(doc_content, encoding="utf-8")

    print(
        f"Generated {args.output_world} and {args.output_doc} "
        f"for N={args.num_drones} in {args.goal_mode} mode."
    )


if __name__ == "__main__":
    main()
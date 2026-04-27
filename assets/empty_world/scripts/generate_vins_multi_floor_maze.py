#!/usr/bin/env python3
"""Generate the multi-floor maze world and UAV task table.

The layout follows a 20m x 20m x 8m shell with a 4m atrium split between
the two upper-floor wings. The script also rewrites uav_position_goal.md so
the 3-drone cross-floor task stays coupled to the world definition.
"""
from __future__ import annotations

import argparse
from pathlib import Path
from typing import List, Optional, Sequence, Tuple


WORLD_NAME = "vins_multi_floor_maze"
OUTPUT_WORLD = Path("worlds/vins_multi_floor_maze.world")
OUTPUT_DOC = Path("uav_position_goal.md")

ROOM_X = 20.0
ROOM_Y = 20.0
ROOM_Z = 8.0
HALF_X = ROOM_X / 2.0
HALF_Y = ROOM_Y / 2.0

FLOOR_THICKNESS = 0.08
WALL_THICKNESS = 0.20
WALL_HEIGHT = 4.0
CEILING_Z = ROOM_Z
LOWER_WALL_CENTER_Z = WALL_HEIGHT / 2.0
UPPER_WALL_CENTER_Z = WALL_HEIGHT + WALL_HEIGHT / 2.0
FLOOR_CENTER_Z = FLOOR_THICKNESS / 2.0
UPPER_PLATFORM_Z = 4.0
UPPER_PLATFORM_THICKNESS = 0.12
UPPER_PLATFORM_CENTER_Z = UPPER_PLATFORM_Z + UPPER_PLATFORM_THICKNESS / 2.0

ATRUIM_X_MIN = 7.5
ATRUIM_X_MAX = 12.5
ATRUIM_WIDTH = ATRUIM_X_MAX - ATRUIM_X_MIN

STEP_LENGTH = 2.0
STEP_WIDTH = 4.0
STEP_HEIGHT = 4.0 / 3.0
STEP_DEPTH_POSITIONS = (3.0, 5.0, 7.0)

PILLAR_RADIUS = 0.25
PILLAR_LENGTH = 4.0

START_POSES: Tuple[Tuple[float, float, float, float], ...] = (
    (2.0, 2.0, 0.5, 0.0),
    (2.0, -2.0, 0.5, 0.0),
    (5.0, 5.0, 5.5, 0.0),
)

GOAL_POSES: Tuple[Tuple[float, float, float, float], ...] = (
    (15.0, 2.0, 0.5, 0.0),
    (15.0, -2.0, 5.5, 0.0),
    (15.0, 5.0, 0.5, 0.0),
)


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


def light_block(name: str, x: float, y: float, z: float, diffuse: str) -> str:
    return f"""
    <light name=\"{name}\" type=\"point\">
      <pose>{fmt(x)} {fmt(y)} {fmt(z)} 0 0 0</pose>
      <diffuse>{diffuse}</diffuse>
      <specular>0.1 0.1 0.1 1</specular>
      <attenuation>
        <range>25</range>
        <constant>0.95</constant>
        <linear>0.03</linear>
        <quadratic>0.003</quadratic>
      </attenuation>
      <cast_shadows>true</cast_shadows>
    </light>
"""


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


def cylinder_model(
    name: str,
    pose: Sequence[float],
    radius: float,
    length: float,
    texture_name: str,
) -> str:
    x, y, z, roll, pitch, yaw = pose
    return f"""
    <model name=\"{name}\">
      <static>true</static>
      <pose>{fmt(x)} {fmt(y)} {fmt(z)} {fmt(roll)} {fmt(pitch)} {fmt(yaw)}</pose>
      <link name=\"link\">
        <collision name=\"collision\">
          <geometry>
            <cylinder>
              <radius>{fmt(radius)}</radius>
              <length>{fmt(length)}</length>
            </cylinder>
          </geometry>
        </collision>
        <visual name=\"visual\">
          <geometry>
            <cylinder>
              <radius>{fmt(radius)}</radius>
              <length>{fmt(length)}</length>
            </cylinder>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>{texture_name}</name>
            </script>
          </material>
        </visual>
      </link>
    </model>
"""


def build_floor_tiles() -> str:
    tiles = []
    half_tile = 5.0
    tile_thickness = FLOOR_THICKNESS
    patterns = [
        (5.0, -5.0, "Gazebo/Checkerboard"),
        (15.0, -5.0, "Gazebo/Grey"),
        (5.0, 5.0, "Gazebo/Grey"),
        (15.0, 5.0, "Gazebo/Checkerboard"),
    ]
    for idx, (x, y, texture) in enumerate(patterns):
        tiles.append(
            box_model(
                f"ground_tile_{idx}",
                (x, y, FLOOR_CENTER_Z, 0.0, 0.0, 0.0),
                (10.0, 10.0, tile_thickness),
                texture,
            )
        )
    return "".join(tiles)


def build_upper_platform() -> str:
    # Implemented as two upper wings around the atrium so the 5m x 20m void
    # remains fully passable for vertical navigation.
    west_wing = box_model(
        "upper_platform_west",
        (3.75, 0.0, UPPER_PLATFORM_CENTER_Z, 0.0, 0.0, 0.0),
        (7.5, ROOM_Y, UPPER_PLATFORM_THICKNESS),
        "Gazebo/WoodFloor",
    )
    east_wing = box_model(
        "upper_platform_east",
        (16.25, 0.0, UPPER_PLATFORM_CENTER_Z, 0.0, 0.0, 0.0),
        (7.5, ROOM_Y, UPPER_PLATFORM_THICKNESS),
        "Gazebo/WoodFloor",
    )
    return west_wing + east_wing


def build_ceiling() -> str:
    return box_model(
        "ceiling_panel",
        (ROOM_X / 2.0, 0.0, ROOM_Z - FLOOR_THICKNESS / 2.0, 0.0, 0.0, 0.0),
        (ROOM_X, ROOM_Y, FLOOR_THICKNESS),
        "Gazebo/Grey",
    )


def build_outer_walls() -> str:
    blocks = []
    lower_texture = "Gazebo/Bricks"
    upper_texture = "Gazebo/PaintedWall"

    lower_specs = [
        ("west_wall_lower", (0.0, 0.0, LOWER_WALL_CENTER_Z, 0.0, 0.0, 0.0), (WALL_THICKNESS, ROOM_Y, WALL_HEIGHT)),
        ("east_wall_lower", (ROOM_X, 0.0, LOWER_WALL_CENTER_Z, 0.0, 0.0, 0.0), (WALL_THICKNESS, ROOM_Y, WALL_HEIGHT)),
        ("south_wall_lower", (ROOM_X / 2.0, -HALF_Y, LOWER_WALL_CENTER_Z, 0.0, 0.0, 0.0), (ROOM_X, WALL_THICKNESS, WALL_HEIGHT)),
        ("north_wall_lower", (ROOM_X / 2.0, HALF_Y, LOWER_WALL_CENTER_Z, 0.0, 0.0, 0.0), (ROOM_X, WALL_THICKNESS, WALL_HEIGHT)),
    ]
    upper_specs = [
        ("west_wall_upper", (0.0, 0.0, UPPER_WALL_CENTER_Z, 0.0, 0.0, 0.0), (WALL_THICKNESS, ROOM_Y, WALL_HEIGHT)),
        ("east_wall_upper", (ROOM_X, 0.0, UPPER_WALL_CENTER_Z, 0.0, 0.0, 0.0), (WALL_THICKNESS, ROOM_Y, WALL_HEIGHT)),
        ("south_wall_upper", (ROOM_X / 2.0, -HALF_Y, UPPER_WALL_CENTER_Z, 0.0, 0.0, 0.0), (ROOM_X, WALL_THICKNESS, WALL_HEIGHT)),
        ("north_wall_upper", (ROOM_X / 2.0, HALF_Y, UPPER_WALL_CENTER_Z, 0.0, 0.0, 0.0), (ROOM_X, WALL_THICKNESS, WALL_HEIGHT)),
    ]

    for name, pose, size in lower_specs:
        blocks.append(box_model(name, pose, size, lower_texture))
    for name, pose, size in upper_specs:
        blocks.append(box_model(name, pose, size, upper_texture))
    return "".join(blocks)


def build_stairs() -> str:
    blocks = []
    for idx, x_center in enumerate(STEP_DEPTH_POSITIONS):
        center_z = STEP_HEIGHT * (idx + 0.5)
        blocks.append(
            box_model(
                f"broad_step_{idx}",
                (x_center, 0.0, center_z, 0.0, 0.0, 0.0),
                (STEP_LENGTH, STEP_WIDTH, STEP_HEIGHT),
                "Gazebo/Wood",
            )
        )
    return "".join(blocks)


def build_pillars() -> str:
    positions = [
        (2.5, -7.5),
        (2.5, 7.5),
        (17.5, -7.5),
        (17.5, 7.5),
    ]
    blocks = []
    for idx, (x, y) in enumerate(positions):
        blocks.append(
            cylinder_model(
                f"upper_support_pillar_{idx}",
                (x, y, PILLAR_LENGTH / 2.0, 0.0, 0.0, 0.0),
                PILLAR_RADIUS,
                PILLAR_LENGTH,
                "Gazebo/Wood",
            )
        )
    return "".join(blocks)


def build_lights() -> str:
    return "".join(
        [
            light_block("lower_point_light", 5.0, 0.0, 2.0, "0.95 0.95 0.90 1"),
            light_block("upper_point_light", 15.0, 0.0, 6.5, "0.90 0.90 0.95 1"),
        ]
    )


def build_spawn_goal_doc() -> str:
    rows = [
        "# UAV 起点与目标点（vins_multi_floor_maze）",
        "",
        "## 3 机跨楼层任务配置",
        "- Drone_0：1 层搜索",
        "- Drone_1：垂直跃迁",
        "- Drone_2：2 层降落",
        "",
        "## 坐标表",
        "| Drone | Start X | Start Y | Start Z | Goal X | Goal Y | Goal Z | Yaw |",
        "|---|---:|---:|---:|---:|---:|---:|---:|",
    ]

    labels = ("Drone_0", "Drone_1", "Drone_2")
    for idx, (start, goal) in enumerate(zip(START_POSES, GOAL_POSES)):
        rows.append(
            f"| {labels[idx]} | {start[0]:.1f} | {start[1]:.1f} | {start[2]:.1f} | {goal[0]:.1f} | {goal[1]:.1f} | {goal[2]:.1f} | {start[3]:.1f} |"
        )

    rows.extend(
        [
            "",
            "## 说明",
            "- Drone_0 保持在 1 层低空穿梭。",
            "- Drone_1 从贯通空域垂直爬升至二层上方。",
            "- Drone_2 从二层下潜回到 1 层。",
            "- 该任务建议使用 same yaw 起始朝向，并由规划器自行处理垂直机动。",
        ]
    )
    return "\n".join(rows) + "\n"


def build_world() -> str:
    return f"""<?xml version=\"1.0\" ?>
<sdf version=\"1.6\">
  <world name=\"{WORLD_NAME}\">
    <gravity>0 0 -9.81</gravity>

    <include>
      <uri>model://sun</uri>
    </include>

    <scene>
      <ambient>0.45 0.45 0.47 1</ambient>
      <background>0.68 0.70 0.74 1</background>
      <shadows>true</shadows>
    </scene>

    <!-- Phase 1: 20m x 20m x 8m shell, mixed floor textures, and atrium void. -->
{build_floor_tiles()}
{build_upper_platform()}
{build_outer_walls()}
{build_ceiling()}

    <!-- Phase 2: broad three-step climb path plus support pillars. -->
{build_stairs()}
{build_pillars()}

    <!-- Phase 3: two-level lighting to reduce exposure jumps during vertical motion. -->
{build_lights()}
  </world>
</sdf>
"""


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Generate vins_multi_floor_maze world and UAV task doc")
    parser.add_argument("--output-world", type=Path, default=OUTPUT_WORLD, help="Output world file")
    parser.add_argument("--output-doc", type=Path, default=OUTPUT_DOC, help="Output markdown file")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    args.output_world.parent.mkdir(parents=True, exist_ok=True)
    args.output_world.write_text(build_world(), encoding="utf-8")
    args.output_doc.write_text(build_spawn_goal_doc(), encoding="utf-8")
    print(f"Generated {args.output_world} and refreshed {args.output_doc}.")


if __name__ == "__main__":
    main()
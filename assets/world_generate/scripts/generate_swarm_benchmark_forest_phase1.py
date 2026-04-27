#!/usr/bin/env python3
"""Generate the Phase 1 world for swarm_benchmark_forest.

This script writes a VINS-friendly 40m x 40m x 5m Gazebo world with:
- checkerboard floor
- textured brick walls
- a configurable circular spawn base around the origin
- per-drone low-profile start and goal anchors placed symmetrically around the origin
"""
from __future__ import annotations

import argparse
from math import atan2, cos, radians, sin
from pathlib import Path

OUT_WORLD = Path("worlds/swarm_benchmark_forest_phase1.world")

ROOM_SIZE_X = 40.0
ROOM_SIZE_Y = 40.0
ROOM_SIZE_Z = 5.0
HALF_X = ROOM_SIZE_X / 2.0
HALF_Y = ROOM_SIZE_Y / 2.0
WALL_H = ROOM_SIZE_Z
WALL_T = 0.4
FLOOR_T = 0.08

DEFAULT_NUM_DRONES = 3
DEFAULT_RADIUS = 15.0
DRONE_Z = 0.10
ANGLE_STEP_DEG = 30.0
MAX_ANGLE_SPAN_DEG = 160.0


def fmt(value: float) -> str:
    return f"{value:.3f}"


def build_angle_sequence(num_drones: int):
    if num_drones <= 1:
        return [0.0]

    span_deg = min(MAX_ANGLE_SPAN_DEG, ANGLE_STEP_DEG * (num_drones - 1))
    step_deg = span_deg / (num_drones - 1)
    start_deg = -span_deg / 2.0
    return [start_deg + i * step_deg for i in range(num_drones)]


def build_spawn_poses(num_drones: int, radius: float):
    spawn_poses = []
    for deg in build_angle_sequence(num_drones):
        th = radians(deg)
        x = radius * cos(th)
        y = radius * sin(th)
        yaw = atan2(-y, -x)
        spawn_poses.append((x, y, DRONE_Z, yaw))
    return spawn_poses


def opposite_pose(spawn_pose):
    x, y, z, yaw = spawn_pose
    return (-x, -y, z, yaw)


def anchor_marker_model(idx: int, pose, suffix: str) -> str:
    x, y, _z, _yaw = pose
    z = 0.12
    size_x = 0.50
    size_y = 0.50
    size_z = 0.06
    if suffix == "start":
        texture_name = "Gazebo/Red"
        ambient = "0.60 0.15 0.15 1"
        diffuse = "0.90 0.20 0.20 1"
    else:
        texture_name = "Gazebo/Green"
        ambient = "0.15 0.60 0.15 1"
        diffuse = "0.20 0.90 0.20 1"

    return f"""
    <model name=\"spawn_anchor_box_{idx}_{suffix}\">
      <static>true</static>
      <pose>{fmt(x)} {fmt(y)} {fmt(z)} 0 0 0.000</pose>
      <link name=\"link\">
        <visual name=\"visual\">
          <geometry>
            <box>
              <size>{fmt(size_x)} {fmt(size_y)} {fmt(size_z)}</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>{texture_name}</name>
            </script>
            <ambient>{ambient}</ambient>
            <diffuse>{diffuse}</diffuse>
            <specular>0.05 0.05 0.05 1</specular>
          </material>
        </visual>
      </link>
    </model>
"""


def box_model(name: str, pose, size, texture_name: str, static: bool = True) -> str:
    x, y, z, yaw = pose
    sx, sy, sz = size
    return f"""
    <model name=\"{name}\">
      <static>{str(static).lower()}</static>
      <pose>{fmt(x)} {fmt(y)} {fmt(z)} 0 0 {fmt(yaw)}</pose>
      <link name=\"link\">
        <collision name=\"collision\">
          <geometry>
            <box>
              <size>{fmt(sx)} {fmt(sy)} {fmt(sz)}</size>
            </box>
          </geometry>
        </collision>
        <visual name=\"visual\">
          <geometry>
            <box>
              <size>{fmt(sx)} {fmt(sy)} {fmt(sz)}</size>
            </box>
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


def floor_model() -> str:
    return f"""
    <model name=\"checker_floor\">
      <static>true</static>
      <pose>0 0 {fmt(FLOOR_T / 2.0)} 0 0 0</pose>
      <link name=\"link\">
        <collision name=\"collision\">
          <geometry>
            <box>
              <size>{fmt(ROOM_SIZE_X)} {fmt(ROOM_SIZE_Y)} {fmt(FLOOR_T)}</size>
            </box>
          </geometry>
        </collision>
        <visual name=\"visual\">
          <geometry>
            <box>
              <size>{fmt(ROOM_SIZE_X)} {fmt(ROOM_SIZE_Y)} {fmt(FLOOR_T)}</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/WoodFloor</name>
            </script>
            <ambient>0.24 0.21 0.16 1</ambient>
            <diffuse>0.50 0.44 0.30 1</diffuse>
            <specular>0.04 0.04 0.04 1</specular>
          </material>
        </visual>
      </link>
    </model>
"""


def wall_models() -> str:
    return "".join(
        [
            box_model(
                "west_wall",
                (-HALF_X + WALL_T / 2.0, 0.0, WALL_H / 2.0, 0.0),
                (WALL_T, ROOM_SIZE_Y, WALL_H),
                "Gazebo/Bricks",
            ),
            box_model(
                "east_wall",
                (HALF_X - WALL_T / 2.0, 0.0, WALL_H / 2.0, 0.0),
                (WALL_T, ROOM_SIZE_Y, WALL_H),
                "Gazebo/Bricks",
            ),
            box_model(
                "south_wall",
                (0.0, -HALF_Y + WALL_T / 2.0, WALL_H / 2.0, 0.0),
                (ROOM_SIZE_X, WALL_T, WALL_H),
                "Gazebo/Bricks",
            ),
            box_model(
                "north_wall",
                (0.0, HALF_Y - WALL_T / 2.0, WALL_H / 2.0, 0.0),
                (ROOM_SIZE_X, WALL_T, WALL_H),
                "Gazebo/Bricks",
            ),
        ]
    )


def anchor_model(idx: int, pose, suffix: str) -> str:
  return anchor_marker_model(idx, pose, suffix)


def build_anchor_models(spawn_poses) -> str:
    blocks = []
    for i, pose in enumerate(spawn_poses):
        blocks.append(anchor_model(i, pose, "start"))
        blocks.append(anchor_model(i, opposite_pose(pose), "goal"))
    return "".join(blocks)


def parse_args():
    parser = argparse.ArgumentParser(description="Generate swarm_benchmark_forest phase 1 world")
    parser.add_argument("--num-drones", type=int, default=DEFAULT_NUM_DRONES, help="Number of drones to support")
    parser.add_argument("--radius", type=float, default=DEFAULT_RADIUS, help="Spawn and goal anchor radius in meters")
    parser.add_argument("--out-world", type=Path, default=OUT_WORLD, help="Output world file")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    spawn_poses = build_spawn_poses(args.num_drones, args.radius)
    anchor_blocks = build_anchor_models(spawn_poses)
    spawn_comment = "\n".join(
        [
            "    <!-- Initial drone start poses (x, y, z, yaw) -->",
            *[
                f"    <!-- drone_{i}: start=({fmt(p[0])}, {fmt(p[1])}, {fmt(p[2])}, {fmt(p[3])}) -->"
                for i, p in enumerate(spawn_poses)
            ],
            "    <!-- Each goal anchor is placed at the symmetric opposite point -->",
            "",
        ]
    )

    world = f"""<?xml version=\"1.0\" ?>
<sdf version=\"1.6\">
  <world name=\"swarm_benchmark_forest\">
    <gravity>0 0 -9.81</gravity>

    <include>
      <uri>model://sun</uri>
    </include>

    <scene>
      <ambient>0.48 0.48 0.48 1</ambient>
      <background>0.7 0.7 0.7 1</background>
      <shadows>true</shadows>
    </scene>

    <!-- Macro world: 40m x 40m x 5m, centered at origin -->
    <model name=\"world_floor\">
      <static>true</static>
      <pose>0 0 {fmt(FLOOR_T / 2.0)} 0 0 0</pose>
      <link name=\"link\">
        <collision name=\"collision\">
          <geometry>
            <box>
              <size>{fmt(ROOM_SIZE_X)} {fmt(ROOM_SIZE_Y)} {fmt(FLOOR_T)}</size>
            </box>
          </geometry>
        </collision>
        <visual name=\"visual\">
          <geometry>
            <box>
              <size>{fmt(ROOM_SIZE_X)} {fmt(ROOM_SIZE_Y)} {fmt(FLOOR_T)}</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/WoodFloor</name>
            </script>
            <ambient>0.24 0.21 0.16 1</ambient>
            <diffuse>0.50 0.44 0.30 1</diffuse>
            <specular>0.04 0.04 0.04 1</specular>
          </material>
        </visual>
      </link>
    </model>

{wall_models()}
{spawn_comment}{anchor_blocks}
  </world>
</sdf>
"""

    out_world: Path = args.out_world
    # If user requested a non-default drone count and didn't override out-world,
    # generate a variant filename with a UAV count suffix (e.g. _6UAV).
    if args.num_drones != DEFAULT_NUM_DRONES and out_world == OUT_WORLD:
      out_world = OUT_WORLD.with_name(f"{OUT_WORLD.stem}_{args.num_drones}UAV{OUT_WORLD.suffix}")

    out_world.parent.mkdir(parents=True, exist_ok=True)
    out_world.write_text(world, encoding="utf-8")
    print(f"Generated {out_world}")
    for i, pose in enumerate(spawn_poses):
        goal = opposite_pose(pose)
        print(
            f"drone_{i}: start=({pose[0]:.3f}, {pose[1]:.3f}, {pose[2]:.3f}, {pose[3]:.3f}) "
            f"goal=({goal[0]:.3f}, {goal[1]:.3f}, {goal[2]:.3f}, {goal[3]:.3f})"
        )


if __name__ == "__main__":
    main()
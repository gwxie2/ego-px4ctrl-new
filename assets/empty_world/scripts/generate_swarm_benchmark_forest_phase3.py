#!/usr/bin/env python3
"""Generate Phase 3 for swarm_benchmark_forest.

This version intentionally adds only more pillars (no actors) to create a dense
static stress-test environment while preserving main-axis passage corridors.
"""
from __future__ import annotations

import argparse
import math
import random
import re
from pathlib import Path
from typing import List, Tuple

DEFAULT_BASE_WORLD = Path("worlds/swarm_benchmark_forest_phase2.world")
DEFAULT_OUT_WORLD = Path("worlds/swarm_benchmark_forest_phase3.world")

ANCHOR_CLEARANCE = 2.0

SEED = 20260321
EXTRA_PILLARS = 45
R_MIN, R_MAX = 0.40, 1.20
HEIGHT = 5.0
MIN_GAP = 0.06

# Keep the main axes more open so three lanes remain passable.
MAIN_AXIS_BANDS = (-5.0, 0.0, 5.0)
CORRIDOR_HALF_WIDTH = 1.35

X_MIN, X_MAX = -18.0, 18.0
Y_MIN, Y_MAX = -18.0, 18.0
MAX_ATTEMPTS = 250000

Existing = Tuple[float, float, float]
Placed = Tuple[float, float, float, float]


def parse_anchor_positions(world_text: str) -> List[Tuple[float, float]]:
  anchors: List[Tuple[float, float]] = []
  pattern = re.compile(r'<model name="spawn_anchor_box_\d+_(?:start|goal)">.*?<pose>([^<]+)</pose>', re.S)
  for m in pattern.finditer(world_text):
    pose = list(map(float, m.group(1).split()))
    anchors.append((pose[0], pose[1]))
  return anchors


def fmt(v: float) -> str:
  return f"{v:.3f}"


def parse_existing_pillars(world_text: str) -> List[Existing]:
  pillars: List[Existing] = []
  pattern = re.compile(
    r'<model name="(?:sparse_pillar|stress_pillar)_\d+">.*?'
    r'<pose>([^<]+)</pose>.*?'
    r'<cylinder>\s*<radius>([^<]+)</radius>\s*<length>([^<]+)</length>',
    re.S,
  )
  for m in pattern.finditer(world_text):
    pose = list(map(float, m.group(1).split()))
    radius = float(m.group(2))
    pillars.append((pose[0], pose[1], radius))
  return pillars


def in_main_axis_corridor(y: float, radius: float) -> bool:
  for axis_y in MAIN_AXIS_BANDS:
    if abs(y - axis_y) < (CORRIDOR_HALF_WIDTH + radius):
      return True
  return False


def too_close(candidate: Placed, others: List[Existing]) -> bool:
  x, y, r, _ = candidate
  for item in others:
    ox, oy, or_ = item[:3]
    if math.hypot(x - ox, y - oy) < (r + or_ + MIN_GAP):
      return True
  return False


def too_close_to_anchors(candidate: Placed, anchors: List[Tuple[float, float]]) -> bool:
  x, y, r, _ = candidate
  for ax, ay in anchors:
    if math.hypot(x - ax, y - ay) < (ANCHOR_CLEARANCE + r):
      return True
  return False


def cylinder_model(idx: int, x: float, y: float, r: float, h: float) -> str:
  z = h / 2.0
  return f"""
  <model name="stress_pillar_{idx:02d}">
    <static>true</static>
    <pose>{fmt(x)} {fmt(y)} {fmt(z)} 0 0 0</pose>
    <link name="link">
    <collision name="collision">
      <geometry>
      <cylinder>
        <radius>{fmt(r)}</radius>
        <length>{fmt(h)}</length>
      </cylinder>
      </geometry>
    </collision>
    <visual name="visual">
      <geometry>
      <cylinder>
        <radius>{fmt(r)}</radius>
        <length>{fmt(h)}</length>
      </cylinder>
      </geometry>
      <material>
      <script>
        <uri>file://media/materials/scripts/gazebo.material</uri>
        <name>Gazebo/Wood</name>
      </script>
      <ambient>0.78 0.63 0.38 1</ambient>
      <diffuse>0.92 0.76 0.45 1</diffuse>
      <specular>0.28 0.28 0.28 1</specular>
      </material>
    </visual>
    </link>
  </model>
"""


def sample_extra_pillars(existing: List[Existing], anchors: List[Tuple[float, float]], count: int, seed: int) -> List[Placed]:
  random.seed(seed)
  placed: List[Placed] = []
  attempts = 0
  while len(placed) < count and attempts < MAX_ATTEMPTS:
    attempts += 1
    r = random.uniform(R_MIN, R_MAX)
    x = random.uniform(X_MIN + r, X_MAX - r)
    y = random.uniform(Y_MIN + r, Y_MAX - r)

    if in_main_axis_corridor(y, r):
      continue

    cand = (x, y, r, HEIGHT)
    if too_close(cand, existing):
      continue
    if too_close(cand, placed):
      continue
    if too_close_to_anchors(cand, anchors):
      continue
    placed.append(cand)

  if len(placed) < count:
    raise RuntimeError(
      f"Only placed {len(placed)}/{count} extra pillars after {attempts} attempts."
    )
  return placed


def parse_args():
  parser = argparse.ArgumentParser(description="Generate swarm_benchmark_forest phase 3 world")
  parser.add_argument("--base-world", type=Path, default=DEFAULT_BASE_WORLD, help="Input phase 2 world file")
  parser.add_argument("--out-world", type=Path, default=DEFAULT_OUT_WORLD, help="Output phase 3 world file")
  parser.add_argument("--seed", type=int, default=SEED, help="Random seed for extra pillar sampling")
  parser.add_argument("--extra-pillars", type=int, default=EXTRA_PILLARS, help="Number of extra stress pillars")
  return parser.parse_args()


def main() -> None:
  args = parse_args()

  if not args.base_world.exists():
    raise FileNotFoundError(f"Missing base world: {args.base_world}")

  content = args.base_world.read_text(encoding="utf-8")
  if "</world>" not in content:
    raise RuntimeError("Invalid SDF: missing </world>")

  existing = parse_existing_pillars(content)
  anchors = parse_anchor_positions(content)
  extras = sample_extra_pillars(existing, anchors, int(args.extra_pillars), int(args.seed))

  pillar_block = "\n    <!-- Phase 3: dense stress-test pillars (no actors) -->\n" + "".join(
    cylinder_model(i, x, y, r, h) for i, (x, y, r, h) in enumerate(extras)
  )

  out = content.replace("\n  </world>", f"{pillar_block}\n  </world>", 1)
  args.out_world.parent.mkdir(parents=True, exist_ok=True)
  args.out_world.write_text(out, encoding="utf-8")

  print(f"Generated {args.out_world} with {len(extras)} extra pillars.")
  print("No actors were added; only static pillars were appended.")
  print("Main axes remain more open around y = -5, 0, 5.")
  print(f"Anchor clearance enforced at {ANCHOR_CLEARANCE:.1f} m around every start/goal anchor.")
  print(f"Stress pillar radius range: [{R_MIN:.2f}, {R_MAX:.2f}] m.")


if __name__ == "__main__":
  main()

#!/usr/bin/env python3
"""Generate Phase 2 for swarm_benchmark_forest.

This script appends sparse static wooden pillars to Phase 1 world while preserving
wide straight corridors for drift evaluation.
"""
import argparse
import math
import re
from pathlib import Path

DEFAULT_BASE_WORLD = Path("worlds/swarm_benchmark_forest_phase1.world")
DEFAULT_OUT_WORLD = Path("worlds/swarm_benchmark_forest_phase2.world")

ANCHOR_CLEARANCE = 2.0
PILLAR_RADIUS_SCALE = 2.0

# The sparse layout is designed in a [5, 35] x [-15, 15] conceptual grid,
# but we re-center it into the 40m world around the origin so pillars remain
# inside the room boundaries.
X_SHIFT = -20.0

PILLARS_RAW = [
  (6.0, -13.0, 0.34, 5.0),
  (10.0, -7.0, 0.40, 5.0),
  (14.0, 13.0, 0.28, 5.0),
  (18.0, -3.0, 0.32, 5.0),
  (22.0, 7.0, 0.36, 5.0),
  (26.0, -13.0, 0.45, 5.0),
  (30.0, 3.0, 0.25, 5.0),
  (34.0, -7.0, 0.38, 5.0),
  (8.0, 13.0, 0.30, 5.0),
  (12.0, -3.0, 0.34, 5.0),
  (16.0, 7.0, 0.29, 5.0),
  (20.0, -13.0, 0.41, 5.0),
  (24.0, 3.0, 0.27, 5.0),
  (28.0, 13.0, 0.36, 5.0),
  (32.0, -3.0, 0.31, 5.0),
  (6.0, 7.0, 0.33, 5.0),
  (18.0, 13.0, 0.42, 5.0),
  (30.0, -13.0, 0.29, 5.0),
]


def fmt(v: float) -> str:
    return f"{v:.3f}"


def cylinder_model(idx: int, x: float, y: float, r: float, h: float) -> str:
    z = h / 2.0
    return f"""
    <model name="sparse_pillar_{idx:02d}">
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
            <ambient>0.75 0.60 0.35 1</ambient>
            <diffuse>0.90 0.72 0.42 1</diffuse>
            <specular>0.30 0.30 0.30 1</specular>
          </material>
        </visual>
      </link>
    </model>
"""


def recenter_pillar(raw_pillar):
    x, y, r, h = raw_pillar
    return x + X_SHIFT, y, r * PILLAR_RADIUS_SCALE, h


def parse_anchor_positions(world_text: str):
  anchors = []
  pattern = re.compile(r'<model name="spawn_anchor_box_\d+_(?:start|goal)">.*?<pose>([^<]+)</pose>', re.S)
  for match in pattern.finditer(world_text):
    pose = list(map(float, match.group(1).split()))
    anchors.append((pose[0], pose[1]))
  return anchors


def violates_anchor_clearance(x: float, y: float, radius: float, anchors) -> bool:
  for ax, ay in anchors:
    if math.hypot(x - ax, y - ay) < (ANCHOR_CLEARANCE + radius):
      return True
  return False


def shift_until_clear(x: float, y: float, radius: float, anchors):
  candidate_x = x
  candidate_y = y
  step = 0.5
  while violates_anchor_clearance(candidate_x, candidate_y, radius, anchors):
    candidate_x += step
    if candidate_x > 18.0:
      candidate_x = -18.0
      candidate_y += 1.0
    if candidate_y > 18.0:
      break
  return candidate_x, candidate_y


def parse_args():
    parser = argparse.ArgumentParser(description="Generate swarm_benchmark_forest phase 2 world")
    parser.add_argument("--base-world", type=Path, default=DEFAULT_BASE_WORLD, help="Input phase 1 world file")
    parser.add_argument("--out-world", type=Path, default=DEFAULT_OUT_WORLD, help="Output phase 2 world file")
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    if not args.base_world.exists():
        raise FileNotFoundError(f"Missing base world: {args.base_world}")

    content = args.base_world.read_text(encoding="utf-8")
    if "</world>" not in content:
        raise RuntimeError("Invalid SDF: missing </world>")

    anchors = parse_anchor_positions(content)
    centered = []
    for raw in PILLARS_RAW:
      x, y, r, h = recenter_pillar(raw)
      x, y = shift_until_clear(x, y, r, anchors)
      centered.append((x, y, r, h))
    pillar_block = "\n    <!-- Phase 2: sparse calibration pillars (re-centered into the room) -->\n" + "".join(
        cylinder_model(i, x, y, r, h) for i, (x, y, r, h) in enumerate(centered)
    )

    out = content.replace("\n  </world>", f"{pillar_block}\n  </world>", 1)
    args.out_world.parent.mkdir(parents=True, exist_ok=True)
    args.out_world.write_text(out, encoding="utf-8")
    print(f"Generated {args.out_world} with {len(centered)} pillars.")
    print("Pillars are re-centered to stay within the 40m x 40m room.")
    print("Wide corridor lanes are preserved around y = -10, 0, 10.")
    print(f"Anchor clearance enforced at {ANCHOR_CLEARANCE:.1f} m around every start/goal anchor.")
    print(f"Sparse pillar radii scaled by {PILLAR_RADIUS_SCALE:.1f}x.")


if __name__ == "__main__":
    main()

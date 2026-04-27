#!/usr/bin/env python3
import math
import random
from pathlib import Path

BASE_WORLD = Path("worlds/vins_ego_forest_stage1.world")
OUTPUT_WORLD = Path("worlds/vins_ego_forest_stage2.world")

SEED = 20260315
CYLINDER_COUNT = 40
RADIUS_MIN = 0.2
RADIUS_MAX = 0.5
HEIGHT = 3.0
MIN_SURFACE_GAP = 0.06

X_MIN, X_MAX = 3.0, 16.0
Y_MIN, Y_MAX = -4.5, 4.5

CORRIDOR_LINES = [0.9, -0.9]
CORRIDOR_HALF_WIDTH = 0.4


def cylinder_model_xml(name: str, x: float, y: float, radius: float) -> str:
    return f"""
    <model name=\"{name}\">
      <static>true</static>
      <pose>{x:.3f} {y:.3f} {HEIGHT / 2.0:.3f} 0 0 0</pose>
      <link name=\"link\">
        <collision name=\"collision\">
          <geometry>
            <cylinder>
              <radius>{radius:.3f}</radius>
              <length>{HEIGHT:.3f}</length>
            </cylinder>
          </geometry>
        </collision>
        <visual name=\"visual\">
          <geometry>
            <cylinder>
              <radius>{radius:.3f}</radius>
              <length>{HEIGHT:.3f}</length>
            </cylinder>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Wood</name>
            </script>
            <ambient>0.70 0.58 0.34 1</ambient>
            <diffuse>0.82 0.68 0.40 1</diffuse>
            <specular>0.25 0.25 0.25 1</specular>
          </material>
        </visual>
      </link>
    </model>
"""


def corridor_safe(y: float, r: float) -> bool:
    for line in CORRIDOR_LINES:
        if abs(y - line) < (r + CORRIDOR_HALF_WIDTH):
            return False
    return True


def sample_layout(count: int):
    random.seed(SEED)
    placed = []
    attempts = 0
    max_attempts = 150000

    while len(placed) < count and attempts < max_attempts:
        attempts += 1
        r = random.uniform(RADIUS_MIN, RADIUS_MAX)
        x = random.uniform(X_MIN + r, X_MAX - r)
        y = random.uniform(Y_MIN + r, Y_MAX - r)

        if not corridor_safe(y, r):
            continue

        valid = True
        for px, py, pr in placed:
            d = math.hypot(x - px, y - py)
            if d < (r + pr + MIN_SURFACE_GAP):
                valid = False
                break

        if valid:
            placed.append((x, y, r))

    if len(placed) < count:
        raise RuntimeError(
            f"Only placed {len(placed)}/{count} cylinders after {attempts} attempts."
        )
    return placed


def main():
    if not BASE_WORLD.exists():
        raise FileNotFoundError(f"Base world not found: {BASE_WORLD}")

    base_content = BASE_WORLD.read_text(encoding="utf-8")
    if "</world>" not in base_content:
        raise RuntimeError("Invalid SDF: missing </world> in base world.")

    cylinders = sample_layout(CYLINDER_COUNT)
    models_xml = "\n    <!-- Phase 2: static cylinder forest -->\n"
    for idx, (x, y, r) in enumerate(cylinders):
        models_xml += cylinder_model_xml(f"forest_cylinder_{idx:02d}", x, y, r)

    stage2_content = base_content.replace("\n  </world>", f"\n{models_xml}\n  </world>")
    OUTPUT_WORLD.write_text(stage2_content, encoding="utf-8")

    print(f"Generated {OUTPUT_WORLD} with {len(cylinders)} cylinders.")


if __name__ == "__main__":
    main()

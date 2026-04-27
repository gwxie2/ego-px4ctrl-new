#!/usr/bin/env python3
from pathlib import Path

BASE_WORLD = Path("worlds/vins_ego_forest_stage2.world")
OUTPUT_WORLD = Path("worlds/vins_ego_forest_stage3.world")

DYNAMIC_BLOCK = r'''
    <!-- Phase 3: dynamic obstacles with scripted trajectories -->
    <actor name="dynamic_obstacle_1">
      <pose>8 -3 0.9 0 0 0</pose>
      <link name="link">
        <inertial>
          <mass>2.0</mass>
          <inertia>
            <ixx>0.2</ixx><ixy>0</ixy><ixz>0</ixz>
            <iyy>0.2</iyy><iyz>0</iyz><izz>0.2</izz>
          </inertia>
        </inertial>
        <collision name="collision">
          <geometry>
            <cylinder>
              <radius>0.30</radius>
              <length>1.8</length>
            </cylinder>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <cylinder>
              <radius>0.30</radius>
              <length>1.8</length>
            </cylinder>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Red</name>
            </script>
          </material>
        </visual>
      </link>
      <script>
        <loop>true</loop>
        <auto_start>true</auto_start>
        <trajectory id="0" type="translation">
          <waypoint><time>0.0</time><pose>8 -3 0.9 0 0 0</pose></waypoint>
          <waypoint><time>6.0</time><pose>8 3 0.9 0 0 0</pose></waypoint>
          <waypoint><time>12.0</time><pose>8 -3 0.9 0 0 0</pose></waypoint>
        </trajectory>
      </script>
    </actor>

    <actor name="dynamic_obstacle_2">
      <pose>13 -3 0.9 0 0 0</pose>
      <link name="link">
        <inertial>
          <mass>2.0</mass>
          <inertia>
            <ixx>0.2</ixx><ixy>0</ixy><ixz>0</ixz>
            <iyy>0.2</iyy><iyz>0</iyz><izz>0.2</izz>
          </inertia>
        </inertial>
        <collision name="collision">
          <geometry>
            <cylinder>
              <radius>0.30</radius>
              <length>1.8</length>
            </cylinder>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <cylinder>
              <radius>0.30</radius>
              <length>1.8</length>
            </cylinder>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Red</name>
            </script>
          </material>
        </visual>
      </link>
      <script>
        <loop>true</loop>
        <auto_start>true</auto_start>
        <trajectory id="0" type="translation">
          <waypoint><time>0.0</time><pose>13 -3 0.9 0 0 0</pose></waypoint>
          <waypoint><time>7.5</time><pose>13 3 0.9 0 0 0</pose></waypoint>
          <waypoint><time>15.0</time><pose>13 -3 0.9 0 0 0</pose></waypoint>
        </trajectory>
      </script>
    </actor>

    <actor name="dynamic_obstacle_3">
      <pose>12 0 0.9 0 0 0</pose>
      <link name="link">
        <inertial>
          <mass>2.0</mass>
          <inertia>
            <ixx>0.2</ixx><ixy>0</ixy><ixz>0</ixz>
            <iyy>0.2</iyy><iyz>0</iyz><izz>0.2</izz>
          </inertia>
        </inertial>
        <collision name="collision">
          <geometry>
            <cylinder>
              <radius>0.28</radius>
              <length>1.8</length>
            </cylinder>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <cylinder>
              <radius>0.28</radius>
              <length>1.8</length>
            </cylinder>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Orange</name>
            </script>
          </material>
        </visual>
      </link>
      <script>
        <loop>true</loop>
        <auto_start>true</auto_start>
        <trajectory id="0" type="translation">
          <waypoint><time>0.0</time><pose>12.0 0.0 0.9 0 0 0</pose></waypoint>
          <waypoint><time>2.0</time><pose>11.414 1.414 0.9 0 0 0</pose></waypoint>
          <waypoint><time>4.0</time><pose>10.0 2.0 0.9 0 0 0</pose></waypoint>
          <waypoint><time>6.0</time><pose>8.586 1.414 0.9 0 0 0</pose></waypoint>
          <waypoint><time>8.0</time><pose>8.0 0.0 0.9 0 0 0</pose></waypoint>
          <waypoint><time>10.0</time><pose>8.586 -1.414 0.9 0 0 0</pose></waypoint>
          <waypoint><time>12.0</time><pose>10.0 -2.0 0.9 0 0 0</pose></waypoint>
          <waypoint><time>14.0</time><pose>11.414 -1.414 0.9 0 0 0</pose></waypoint>
          <waypoint><time>16.0</time><pose>12.0 0.0 0.9 0 0 0</pose></waypoint>
        </trajectory>
      </script>
    </actor>
'''


def main():
    if not BASE_WORLD.exists():
        raise FileNotFoundError(f"Base world not found: {BASE_WORLD}")

    base_content = BASE_WORLD.read_text(encoding="utf-8")
    if "</world>" not in base_content:
        raise RuntimeError("Invalid SDF: missing </world> in stage2 world.")

    stage3_content = base_content.replace("\n  </world>", f"\n{DYNAMIC_BLOCK}\n  </world>")
    OUTPUT_WORLD.write_text(stage3_content, encoding="utf-8")
    print(f"Generated {OUTPUT_WORLD} with 3 dynamic obstacles.")


if __name__ == "__main__":
    main()

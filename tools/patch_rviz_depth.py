import re

file_path = '/home/guanwen/XTDrone/cleanroom_ws/src/clean_uav_core/rviz/phase2_ego_obstacle_debug.rviz'
with open(file_path, 'r') as f:
    content = f.read()

# Fix visual illusion in depth image rendering: disable normalize and set strict boundaries
content = re.sub(r'Normalize Range: true', 'Normalize Range: false', content)
content = re.sub(r'Max Value: 1\s', 'Max Value: 8\n', content)
content = re.sub(r'Min Value: 0\s', 'Min Value: 0\n', content)

with open(file_path, 'w') as f:
    f.write(content)

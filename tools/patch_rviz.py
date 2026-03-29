import re

file_path = '/home/guanwen/XTDrone/cleanroom_ws/src/clean_uav_core/rviz/phase2_ego_obstacle_debug.rviz'
with open(file_path, 'r') as f:
    content = f.read()

# Replace QMainWindow State
new_state = "000000ff00000000fd0000000300000000000000fa00000000fc0200000001fb000000100044006900730070006c0061007900730100000000ffffffff0000001e00ffffff00000001000000000000037afc0200000003fc00000000ffffffff0000004a0100002bfa000000020200000003fb0000001200530065006c0065006300740069006f006e0100000000ffffffff0000001e00fffffffb0000001e0054006f006f006c002000500072006f00700065007200740069006500730100000000ffffffff0000001e00fffffffb0000000a005600690065007700730100000000ffffffff0000001e00fffffffb0000001400440065007000740068002000750061007600300100000000000001900000001e00fffffffb0000001400440065007000740068002000750061007600310100000000000001900000001e00ffffff000000030000000000000000fc0100000001fb0000000800540069006d00650100000000ffffffff0000004500ffffff000000000000000000000004000000040000000800000008fc00000000"

content = re.sub(r'QMainWindow State: [0-9a-f]+', f'QMainWindow State: {new_state}', content)

# Make sure these are added to layout properly
if "Depth uav0:" not in content:
    content += "\n  Depth uav0:\n    collapsed: false"
if "Depth uav1:" not in content:
    content += "\n  Depth uav1:\n    collapsed: false"

# Increase height and width to look good out of the box
content = re.sub(r'  Height: \d+', '  Height: 1080', content)
content = re.sub(r'  Width: \d+', '  Width: 1920', content)

# Also let's edit the Displays Tree Splitter Ratio so the text inside the thin Displays panel doesn't get cut off weirdly
content = re.sub(r'Splitter Ratio: \d+\.\d+', 'Splitter Ratio: 0.5', content)

with open(file_path, 'w') as f:
    f.write(content)

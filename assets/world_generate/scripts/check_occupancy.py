#!/usr/bin/env python3
"""
检查指定 world 文件中给定点 (x,y,z) 是否落在任意 collision 几何体内。
使用方式：直接运行，会检查 (18,2.5,0.1) 与 (18,-2.5,0.1)
"""
import re
import math
from pathlib import Path

WORLD = Path('worlds/vins_ego_forest_stage2.world')
CHECK_POINTS = [ (18.0, 2.5, 0.1), (18.0, -2.5, 0.1) ]
TOL = 1e-9


def parse_models(text):
    models = []
    for m in re.finditer(r'<model name="([^"]+)">(.*?)</model>', text, re.S):
        name = m.group(1)
        body = m.group(2)
        pose_m = re.search(r'<pose>([^<]+)</pose>', body)
        if pose_m:
            pose = list(map(float, pose_m.group(1).split()))
        else:
            pose = [0.0]*6
        px, py, pz = pose[0], pose[1], pose[2]

        # find collision geometries
        geoms = []
        for c in re.finditer(r'<collision[^>]*>(.*?)</collision>', body, re.S):
            cbody = c.group(1)
            # box
            box_m = re.search(r'<box>.*?<size>([^<]+)</size>.*?</box>', cbody, re.S)
            if box_m:
                sx, sy, sz = map(float, box_m.group(1).split())
                geoms.append(('box', (sx, sy, sz), (px, py, pz)))
                continue
            # cylinder
            cyl_m = re.search(r'<cylinder>.*?<radius>([^<]+)</radius>.*?<length>([^<]+)</length>.*?</cylinder>', cbody, re.S)
            if cyl_m:
                r = float(cyl_m.group(1)); length = float(cyl_m.group(2))
                geoms.append(('cylinder', (r, length), (px, py, pz)))
                continue
            # fallback: look for geometry with cylinder/box in visual too
        models.append({'name': name, 'pose': (px,py,pz), 'geoms': geoms})
    return models


def point_in_box(x,y,z, box_center, size):
    cx,cy,cz = box_center
    sx,sy,sz = size
    if abs(x-cx) <= sx/2.0 + TOL and abs(y-cy) <= sy/2.0 + TOL and abs(z-cz) <= sz/2.0 + TOL:
        return True
    return False


def point_in_cylinder(x,y,z, cyl_center, params):
    cx,cy,cz = cyl_center
    r, length = params
    # cylinder axis aligned with z
    horiz = math.hypot(x-cx, y-cy)
    if horiz <= r + TOL and abs(z-cz) <= length/2.0 + TOL:
        return True
    return False


def check_point(models, pt):
    x,y,z = pt
    occupied = []
    for m in models:
        for geom in m['geoms']:
            if geom[0] == 'box':
                size = geom[1]
                center = geom[2]
                if point_in_box(x,y,z, center, size):
                    occupied.append((m['name'], 'box', center, size))
            elif geom[0] == 'cylinder':
                params = geom[1]
                center = geom[2]
                if point_in_cylinder(x,y,z, center, params):
                    occupied.append((m['name'], 'cylinder', center, params))
    return occupied


def main():
    if not WORLD.exists():
        print('World file not found:', WORLD)
        return 2
    text = WORLD.read_text(encoding='utf-8')
    models = parse_models(text)

    for pt in CHECK_POINTS:
        occ = check_point(models, pt)
        if occ:
            print(f'POINT {pt} OCCUPIED by {len(occ)} object(s):')
            for o in occ:
                print('  -', o[0], o[1], o[2], o[3])
        else:
            print(f'POINT {pt} is FREE (no collision contains it).')

if __name__ == '__main__':
    main()

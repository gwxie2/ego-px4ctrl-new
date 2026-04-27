#!/usr/bin/env python3
"""
将 Gazebo World 文件中的所有 model、actor、wall 等坐标和几何尺寸按指定比例等比例放大。
放大中心为原点 (0,0,0)。
"""
import argparse
import re
from pathlib import Path

DEFAULT_SCALE = 2.0
DEFAULT_SUFFIX = "_x2"
SKIP_SUFFIX_PATTERN = re.compile(r'_x\d+(?:\.\d+)?')


def scale_pose(match, scale):
    pose_str = match.group(1).strip()
    parts = list(map(float, pose_str.split()))
    for i in range(3):
        parts[i] *= scale
    new_pose_str = " ".join(f"{p:g}" for p in parts)
    return f"<pose>{new_pose_str}</pose>"


def scale_size(match, scale):
    size_str = match.group(1).strip()
    parts = list(map(float, size_str.split()))
    scaled_parts = [p * scale for p in parts]
    new_size_str = " ".join(f"{p:g}" for p in scaled_parts)
    return f"<size>{new_size_str}</size>"


def scale_radius(match, scale):
    r = float(match.group(1)) * scale
    return f"<radius>{r:g}</radius>"


def scale_length(match, scale):
    l = float(match.group(1)) * scale
    return f"<length>{l:g}</length>"


def scale_world_file(input_path, output_path, scale):
    if not input_path.exists():
        print(f"Skip: {input_path} not found")
        return

    content = input_path.read_text(encoding='utf-8')
    content = re.sub(r'<pose\b[^>]*>([^<]+)</pose>', lambda m: scale_pose(m, scale), content)
    content = re.sub(r'<size\b[^>]*>([^<]+)</size>', lambda m: scale_size(m, scale), content)
    content = re.sub(r'<radius\b[^>]*>([^<]+)</radius>', lambda m: scale_radius(m, scale), content)
    content = re.sub(r'<length\b[^>]*>([^<]+)</length>', lambda m: scale_length(m, scale), content)

    output_path.write_text(content, encoding='utf-8')
    print(f"Generated scaled world: {output_path}")


def make_output_path(input_path, suffix):
    return input_path.with_name(f"{input_path.stem}{suffix}{input_path.suffix}")


def main():
    parser = argparse.ArgumentParser(description='Scale Gazebo world files to a given factor.')
    parser.add_argument('--scale', type=float, default=DEFAULT_SCALE, help='Scale factor for geometry and poses.')
    parser.add_argument('--suffix', default=DEFAULT_SUFFIX, help='Suffix to add to generated world filenames.')
    parser.add_argument('--world-dir', default='worlds', help='Directory containing world files.')
    args = parser.parse_args()

    world_dir = Path(args.world_dir)
    if not world_dir.exists():
        raise SystemExit(f"World directory not found: {world_dir}")

    for input_path in sorted(world_dir.glob('*.world')):
        if SKIP_SUFFIX_PATTERN.search(input_path.stem):
            continue
        output_path = make_output_path(input_path, args.suffix)
        scale_world_file(input_path, output_path, args.scale)

if __name__ == '__main__':
    main()

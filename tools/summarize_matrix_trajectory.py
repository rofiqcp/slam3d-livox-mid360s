#!/usr/bin/env python3
import math
import pathlib
import sys


def matmul3(a, b):
    return [[sum(a[r][k] * b[k][c] for k in range(3)) for c in range(3)] for r in range(3)]


def transpose3(a):
    return [[a[c][r] for c in range(3)] for r in range(3)]


def rotation_angle(a, b):
    rel = matmul3(transpose3(a), b)
    tr = rel[0][0] + rel[1][1] + rel[2][2]
    c = max(-1.0, min(1.0, (tr - 1.0) * 0.5))
    return math.acos(c)


def parse_pose(line):
    v = [float(x) for x in line.split()]
    if len(v) != 12:
        raise ValueError(f"expected 12 scalars, got {len(v)}")
    r = [
        [v[0], v[1], v[2]],
        [v[4], v[5], v[6]],
        [v[8], v[9], v[10]],
    ]
    t = (v[3], v[7], v[11])
    return r, t


def dist(a, b):
    return math.sqrt(sum((a[i] - b[i]) ** 2 for i in range(3)))


if len(sys.argv) != 3:
    print("usage: summarize_matrix_trajectory.py estimate.txt summary.yaml", file=sys.stderr)
    sys.exit(2)

src = pathlib.Path(sys.argv[1])
dst = pathlib.Path(sys.argv[2])
poses = [parse_pose(line) for line in src.read_text().splitlines() if line.strip()]
if not poses:
    raise SystemExit("trajectory is empty")

path_len = 0.0
max_step = 0.0
max_rot = 0.0
for i in range(1, len(poses)):
    step = dist(poses[i - 1][1], poses[i][1])
    path_len += step
    max_step = max(max_step, step)
    max_rot = max(max_rot, rotation_angle(poses[i - 1][0], poses[i][0]))

z = [p[1][2] for p in poses]
disp = dist(poses[0][1], poses[-1][1])

dst.parent.mkdir(parents=True, exist_ok=True)
dst.write_text(
    "backend: mad_icp\n"
    f"poses: {len(poses)}\n"
    f"path_length_m: {path_len:.9f}\n"
    f"displacement_m: {disp:.9f}\n"
    f"z_span_m: {max(z) - min(z):.9f}\n"
    f"max_step_m: {max_step:.9f}\n"
    f"max_step_angle_rad: {max_rot:.9f}\n"
    f"final_xyz: [{poses[-1][1][0]:.9f}, {poses[-1][1][1]:.9f}, {poses[-1][1][2]:.9f}]\n"
)

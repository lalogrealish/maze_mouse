# -*- coding: utf-8 -*-
"""离线校验工具：验证迷宫几何 + 验证左墙跟随算法能否走到出口。

原理：
  1) 把每面墙（盒子）拆成 4 条线段，以机器人位姿为原点向 360 个方向发射射线，
     求最近交点即得 /scan 的 ranges 读数；
  2) 把读数送进与 ROS 节点完全相同的 maze_mouse_core.scan_zones() 与
     CellWallFollower.step()；
  3) 按差速运动学积分位姿，统计最小离墙距离（判断会不会擦墙），
     最后用 Pillow 把墙体与轨迹画成 PNG。

依赖：numpy、Pillow
    sudo apt install python3-numpy python3-pil

用法：
    # 先从 ROS2 侧导出布局 json（含墙体盒子与线段）
    ros2 run maze_mouse gen_maze_world --out /tmp/maze.world --json /tmp/maze_layout.json
    # 再跑离线校验
    python3 validate_maze.py /tmp/maze_layout.json /tmp/maze_run.png
    python3 validate_maze.py /tmp/maze_layout.json /tmp/maze_run.png -v
"""
import json
import math
import os
import sys

import numpy as np
from PIL import Image, ImageDraw

# ---- 定位本仓库里的算法源码（与 02_参考代码 保持相对位置） ----
HERE = os.path.dirname(os.path.abspath(__file__))
PKG_CANDIDATES = [
    os.path.join(HERE, '..', '02_参考代码', 'maze_ws', 'src', 'maze_mouse'),
    os.path.join(HERE, '..', 'maze_ws', 'src', 'maze_mouse'),
]
for cand in PKG_CANDIDATES:
    if os.path.isdir(os.path.join(cand, 'maze_mouse')):
        sys.path.insert(0, os.path.abspath(cand))
        break

from maze_mouse.maze_mouse_core import CellWallFollower, scan_zones  # noqa: E402

N_RAYS = 360
ANGLE_MIN = -math.pi
ANGLE_INC = 2.0 * math.pi / N_RAYS
RANGE_MAX = 3.5
ROBOT_R = 0.105


def load(path):
    with open(path, 'r', encoding='utf-8') as f:
        return json.load(f)


def raycast(pose, seg_a, seg_b):
    """返回 (N_RAYS,) 的测距数组。pose=(x, y, yaw)。"""
    x, y, yaw = pose
    ang = ANGLE_MIN + np.arange(N_RAYS) * ANGLE_INC + yaw
    d = np.stack([np.cos(ang), np.sin(ang)], axis=1)
    o = np.array([x, y])

    a = seg_a
    e = seg_b - a
    ao = a[None, :, :] - o[None, None, :]
    dd = d[:, None, :]
    ee = e[None, :, :]

    denom = dd[..., 0] * ee[..., 1] - dd[..., 1] * ee[..., 0]
    num_t = ao[..., 0] * ee[..., 1] - ao[..., 1] * ee[..., 0]
    num_s = ao[..., 0] * dd[..., 1] - ao[..., 1] * dd[..., 0]

    with np.errstate(divide='ignore', invalid='ignore'):
        t = num_t / denom
        s = num_s / denom
    valid = (np.abs(denom) > 1e-12) & (t > 0.0) & (s >= 0.0) & (s <= 1.0)
    t = np.where(valid, t, np.inf)
    r = t.min(axis=1)
    r = np.clip(r, 0.0, RANGE_MAX)
    r[r > RANGE_MAX] = np.inf
    return r


def run(layout, dt=0.05, t_max=180.0, verbose=False):
    segs = np.array(layout['segments'], dtype=float)
    seg_a, seg_b = segs[:, 0:2], segs[:, 2:4]

    x0, y0, _ = layout['spawn']
    exit_x_odo = layout['exit_x_world'] - x0

    ctrl = CellWallFollower()
    pose = [x0, y0, layout['spawn'][2]]
    path = [tuple(pose)]
    t = 0.0
    min_clear = 9.9
    bad = 0
    last_state = None

    while t < t_max:
        r = raycast(pose, seg_a, seg_b)
        z = scan_zones(r, ANGLE_MIN, ANGLE_INC, max_range=RANGE_MAX)
        odom = (pose[0] - x0, pose[1] - y0, pose[2])
        v, w, st = ctrl.step(odom, z, dt)

        if verbose and st != last_state:
            print(f'  t={t:6.1f} {st:7s} world=({pose[0]:+.2f},{pose[1]:+.2f}) '
                  f'cell={ctrl.cell} F={z["front"]:.2f} L={z["left"]:.2f} '
                  f'R={z["right"]:.2f} {ctrl.last_decision}')
            last_state = st

        m = float(np.nanmin(r))
        min_clear = min(min_clear, m)
        if m < ROBOT_R * 0.9:
            bad += 1

        pose[0] += v * math.cos(pose[2]) * dt
        pose[1] += v * math.sin(pose[2]) * dt
        pose[2] += w * dt
        t += dt
        path.append(tuple(pose))

        if pose[0] - x0 > exit_x_odo:
            return dict(ok=True, t=t, pose=pose, path=path, min_clear=min_clear,
                        hits=bad, ctrl=ctrl)
    return dict(ok=False, t=t, pose=pose, path=path, min_clear=min_clear,
                hits=bad, ctrl=ctrl)


def draw(layout, res, out_png):
    scale = 170
    margin = 0.6
    boxes = layout['boxes']
    xs = [b[0] - b[2] / 2 for b in boxes] + [b[0] + b[2] / 2 for b in boxes]
    ys = [b[1] - b[3] / 2 for b in boxes] + [b[1] + b[3] / 2 for b in boxes]
    x_min, x_max = min(xs) - margin, max(xs) + margin
    y_min, y_max = min(ys) - margin, max(ys) + margin
    W = int((x_max - x_min) * scale)
    H = int((y_max - y_min) * scale)

    def to_px(x, y):
        return ((x - x_min) * scale, H - (y - y_min) * scale)

    img = Image.new('RGB', (W, H), (245, 245, 248))
    d = ImageDraw.Draw(img)
    for cx, cy, sx, sy in boxes:
        x1, y1 = to_px(cx - sx / 2, cy + sy / 2)
        x2, y2 = to_px(cx + sx / 2, cy - sy / 2)
        d.rectangle([x1, y1, x2, y2], fill=(70, 72, 84))

    pts = [to_px(px, py) for px, py, _ in res['path']]
    if len(pts) > 1:
        d.line(pts, fill=(220, 40, 40), width=3)

    sx, sy = to_px(layout['spawn'][0], layout['spawn'][1])
    d.ellipse([sx - 7, sy - 7, sx + 7, sy + 7], fill=(20, 160, 60))
    ex, ey = to_px(res['pose'][0], res['pose'][1])
    d.ellipse([ex - 7, ey - 7, ex + 7, ey + 7], fill=(40, 90, 220))
    img.save(out_png)
    print('图片已保存:', out_png, img.size)


def main():
    if len(sys.argv) < 3:
        print(__doc__)
        return 1
    layout = load(sys.argv[1])
    out_png = sys.argv[2]
    verbose = len(sys.argv) > 3 and sys.argv[3] == '-v'
    res = run(layout, verbose=verbose)
    print(f"结果: {'成功' if res['ok'] else '失败'}  用时 {res['t']:.1f}s  "
          f"终点 ({res['pose'][0]:.2f}, {res['pose'][1]:.2f})  "
          f"最近距离 {res['min_clear']:.3f}m  贴墙帧数 {res['hits']}")
    ctrl = res['ctrl']
    print(f"决策步数 {len(ctrl.history)}，"
          f"转向次数 {sum(1 for h in ctrl.history if h[2] != 'front_open')}")
    print('路线:', ' '.join(f'{h[0]}{h[1]}' for h in ctrl.history))
    draw(layout, res, out_png)
    return 0


if __name__ == '__main__':
    sys.exit(main())

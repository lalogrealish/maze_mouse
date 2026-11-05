#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""迷宫 world 生成器：用"迭代 DFS"生成完美迷宫（无环生成树），并输出 Gazebo world。

为什么用完美迷宫（perfect maze）？
  左手法则（left-hand rule）能保证走出迷宫的数学前提是"迷宫连通且无环"，
  即任意两格之间只有唯一通路。DFS 生成的生成树天然满足这个条件，
  所以本任务的左墙跟随一定能到出口 —— 这是报告里可以写的一段理论分析。

为什么用脚本而不用手工摆墙？
  5x5 迷宫有 60 段内墙，手写容易漏墙、错位；脚本可以参数化（行列数、种子），
  还能顺带做"连通性自检"，这是工程化的做法。

用法：
    ros2 run maze_mouse gen_maze_world                       # 生成 ./maze.world
    ros2 run maze_mouse gen_maze_world --out /tmp/maze.world --cols 6 --rows 6
    ros2 run maze_mouse gen_maze_world --check               # 只做连通性自检，不写文件
"""

import argparse
import json
import math
import random
import sys
from collections import deque

# 四个方向：(dx, dy, 名字)
DIRS = [(1, 0), (0, 1), (-1, 0), (0, -1)]


# ----------------------------------------------------------------------------
# 1. 完美迷宫生成
# ----------------------------------------------------------------------------
def generate_maze(cols, rows, seed):
    """返回 (v_wall, h_wall)。

    v_wall[i][j] : 位于 x = i*pitch 的竖直墙，覆盖第 j 行（i 取 0..cols，含两侧边界）
    h_wall[i][j] : 位于 y = j*pitch 的水平墙，覆盖第 i 列（j 取 0..rows，含两侧边界）
    True 表示有墙，False 表示已打通。
    """
    v_wall = [[True] * rows for _ in range(cols + 1)]
    h_wall = [[True] * (rows + 1) for _ in range(cols)]

    rnd = random.Random(seed)
    visited = [[False] * rows for _ in range(cols)]
    stack = [(0, 0)]
    visited[0][0] = True

    while stack:
        x, y = stack[-1]
        nbrs = []
        for dx, dy in DIRS:
            nx, ny = x + dx, y + dy
            if 0 <= nx < cols and 0 <= ny < rows and not visited[nx][ny]:
                nbrs.append((nx, ny, dx, dy))
        if not nbrs:
            stack.pop()
            continue
        nx, ny, dx, dy = rnd.choice(nbrs)
        visited[nx][ny] = True
        if dx == 1:      # 向东打通 -> 去掉 x = (x+1)*pitch 的竖墙
            v_wall[x + 1][y] = False
        elif dx == -1:
            v_wall[x][y] = False
        elif dy == 1:    # 向北打通 -> 去掉 y = (y+1)*pitch 的横墙
            h_wall[x][y + 1] = False
        else:
            h_wall[x][y] = False
        stack.append((nx, ny))

    return v_wall, h_wall


def check_maze(v_wall, h_wall, cols, rows, entry_row, exit_row):
    """自检：1) 所有格子连通；2) 无环（边数 = 格数-1）；3) 入口/出口可达。"""
    def neighbors(i, j):
        out = []
        if not v_wall[i + 1][j]:
            out.append((i + 1, j))
        if not v_wall[i][j]:
            out.append((i - 1, j))
        if not h_wall[i][j + 1]:
            out.append((i, j + 1))
        if not h_wall[i][j]:
            out.append((i, j - 1))
        return [(a, b) for a, b in out if 0 <= a < cols and 0 <= b < rows]

    seen = {(0, 0)}
    q = deque([(0, 0)])
    edges = 0
    while q:
        i, j = q.popleft()
        for a, b in neighbors(i, j):
            edges += 1
            if (a, b) not in seen:
                seen.add((a, b))
                q.append((a, b))
    edges //= 2
    ok = (len(seen) == cols * rows) and (edges == cols * rows - 1)
    path_ok = (cols - 1, exit_row) in seen or (0, exit_row) in seen
    return ok, len(seen), edges, cols * rows, path_ok


def bfs_path(v_wall, h_wall, cols, rows, start, goal):
    """返回从 start 到 goal 的格子路径（用于文档里的参考路线，非机器人导航用）。"""
    def neighbors(i, j):
        out = []
        if not v_wall[i + 1][j]:
            out.append((i + 1, j))
        if not v_wall[i][j]:
            out.append((i - 1, j))
        if not h_wall[i][j + 1]:
            out.append((i, j + 1))
        if not h_wall[i][j]:
            out.append((i, j - 1))
        return [(a, b) for a, b in out if 0 <= a < cols and 0 <= b < rows]

    prev = {start: None}
    q = deque([start])
    while q:
        cur = q.popleft()
        if cur == goal:
            break
        for nb in neighbors(*cur):
            if nb not in prev:
                prev[nb] = cur
                q.append(nb)
    if goal not in prev:
        return []
    path = []
    cur = goal
    while cur is not None:
        path.append(cur)
        cur = prev[cur]
    return path[::-1]


# ----------------------------------------------------------------------------
# 2. 把墙转成 SDF 盒体（合并同类相邻墙，减少实体数量）
# ----------------------------------------------------------------------------
def merge_vertical(v_wall, cols, rows, pitch, thickness):
    boxes = []   # (cx, cy, size_x, size_y)
    for i in range(cols + 1):
        j = 0
        while j < rows:
            if not v_wall[i][j]:
                j += 1
                continue
            j0 = j
            while j + 1 < rows and v_wall[i][j + 1]:
                j += 1
            j1 = j
            y0 = j0 * pitch - (thickness / 2.0 if j0 == 0 else 0.0)
            y1 = (j1 + 1) * pitch + (thickness / 2.0 if j1 == rows - 1 else 0.0)
            boxes.append((i * pitch, (y0 + y1) / 2.0, thickness, y1 - y0))
            j += 1
    return boxes


def merge_horizontal(h_wall, cols, rows, pitch, thickness):
    boxes = []
    for j in range(rows + 1):
        i = 0
        while i < cols:
            if not h_wall[i][j]:
                i += 1
                continue
            i0 = i
            while i + 1 < cols and h_wall[i + 1][j]:
                i += 1
            i1 = i
            x0 = i0 * pitch - (thickness / 2.0 if i0 == 0 else 0.0)
            x1 = (i1 + 1) * pitch + (thickness / 2.0 if i1 == cols - 1 else 0.0)
            boxes.append(((x0 + x1) / 2.0, j * pitch, x1 - x0, thickness))
            i += 1
    return boxes


def box_to_segments(cx, cy, sx, sy):
    """把矩形墙拆成 4 条线段，供离线射线仿真使用。"""
    hx, hy = sx / 2.0, sy / 2.0
    p = [(cx - hx, cy - hy), (cx + hx, cy - hy),
         (cx + hx, cy + hy), (cx - hx, cy + hy)]
    return [(p[k][0], p[k][1], p[(k + 1) % 4][0], p[(k + 1) % 4][1]) for k in range(4)]


# ----------------------------------------------------------------------------
# 3. 输出 SDF
# ----------------------------------------------------------------------------
WALL_MAT = """<material>
              <ambient>0.45 0.45 0.52 1</ambient>
              <diffuse>0.55 0.55 0.62 1</diffuse>
              <specular>0.1 0.1 0.1 1</specular>
            </material>"""

# 显式接触参数（关键！）
# Gazebo 默认 kp=1e12，与 1 ms 步长叠加会让接触求解器产生持续抖动，
# 表现为"什么都没发指令，小车却以 5 cm/s 匀速往后爬"——实测踩过的坑。
# 把 kp 降到 1e6、kd 调到 100，并给出 mu，接触就稳定了。
SURFACE = """<surface>
              <friction>
                <ode>
                  <mu>1.0</mu>
                  <mu2>1.0</mu2>
                </ode>
              </friction>
              <contact>
                <ode>
                  <kp>1000000.0</kp>
                  <kd>100.0</kd>
                  <max_vel>0.01</max_vel>
                  <min_depth>0.001</min_depth>
                </ode>
              </contact>
            </surface>"""


def wall_model(name, cx, cy, sx, sy, height):
    return f"""    <model name="{name}">
      <static>true</static>
      <pose>{cx:.4f} {cy:.4f} {height / 2.0:.4f} 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>{sx:.4f} {sy:.4f} {height:.4f}</size></box></geometry>
          {SURFACE}
        </collision>
        <visual name="visual">
          <geometry><box><size>{sx:.4f} {sy:.4f} {height:.4f}</size></box></geometry>
          {WALL_MAT}
        </visual>
      </link>
    </model>"""


def marker_model(name, cx, cy, radius, rgba, height=0.003):
    r, g, b, a = rgba
    return f"""    <model name="{name}">
      <static>true</static>
      <pose>{cx:.4f} {cy:.4f} {height / 2.0 + 0.001:.4f} 0 0 0</pose>
      <link name="link">
        <visual name="visual">
          <geometry><cylinder><radius>{radius:.4f}</radius><length>{height:.4f}</length></cylinder></geometry>
          <material>
            <ambient>{r} {g} {b} {a}</ambient>
            <diffuse>{r} {g} {b} {a}</diffuse>
            <emissive>{r * 0.4} {g * 0.4} {b * 0.4} {a}</emissive>
          </material>
        </visual>
      </link>
    </model>"""


def build_world(boxes, cols, rows, pitch, thickness, wall_height,
                spawn, exit_world, entry_row):
    maze_w = cols * pitch
    maze_h = rows * pitch
    cx_maze, cy_maze = maze_w / 2.0, maze_h / 2.0
    parts = []
    parts.append(f"""<?xml version="1.0" ?>
<!--
  迷宫仿真场景（由 gen_maze_world.py 自动生成，请勿手工修改）
  规模：{cols} x {rows} 格，格距 {pitch:.2f} m（通道 {pitch - thickness:.2f} m + 墙厚 {thickness:.2f} m）
  外框：x ∈ [0, {maze_w:.2f}]，y ∈ [0, {maze_h:.2f}]
  入口：西墙第 {entry_row} 行（y ≈ {entry_row * pitch + pitch / 2:.2f}）
  出口：东墙
-->
<sdf version="1.6">
  <world name="maze_world">

    <!-- 物理引擎：步长 1 ms，保证 0.16 m/s 低速下碰撞检测稳定 -->
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_update_rate>1000</real_time_update_rate>
      <ode>
        <solver>
          <type>quick</type>
          <iters>50</iters>
        </solver>
      </ode>
    </physics>

    <scene>
      <ambient>0.55 0.55 0.6 1</ambient>
      <background>0.25 0.25 0.30 1</background>
      <shadows>1</shadows>
    </scene>

    <!-- 光照：内嵌平行光，不依赖 Gazebo 在线模型库（离线也能跑） -->
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.15 0.15 0.15 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.35 -0.9</direction>
    </light>

    <!-- 地面：一个大盒子，上表面正好在 z = 0 -->
    <model name="ground">
      <static>true</static>
      <pose>{cx_maze:.4f} {cy_maze:.4f} -0.05 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>12 12 0.1</size></box></geometry>
          {SURFACE}
        </collision>
        <visual name="visual">
          <geometry><box><size>12 12 0.1</size></box></geometry>
          <material>
            <ambient>0.75 0.75 0.78 1</ambient>
            <diffuse>0.75 0.75 0.78 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- ===== 迷宫地面色块（仅视觉，无碰撞，抬高 1 mm 避免与地面闪烁） ===== -->
    <model name="maze_floor">
      <static>true</static>
      <pose>{cx_maze:.4f} {cy_maze:.4f} 0.001 0 0 0</pose>
      <link name="link">
        <visual name="visual">
          <geometry><box><size>{maze_w:.4f} {maze_h:.4f} 0.002</size></box></geometry>
          <material>
            <ambient>0.85 0.85 0.88 1</ambient>
            <diffuse>0.85 0.85 0.88 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- Gazebo ROS 状态插件：提供 /gazebo/model_states 与 /get_entity_state 服务，
         用来在调试时读取机器人的"真实世界位姿"，和里程计读数做对照。
         报告里"里程计标定/误差分析"那一节就靠它。 -->
    <plugin name="gazebo_ros_state" filename="libgazebo_ros_state.so">
      <ros>
        <namespace>/</namespace>
      </ros>
      <update_rate>30.0</update_rate>
    </plugin>

    <!-- ===== 迷宫墙体 ===== -->""")

    for k, (cx, cy, sx, sy) in enumerate(boxes):
        parts.append(wall_model(f'wall_{k:03d}', cx, cy, sx, sy, wall_height))

    parts.append(f"""
    <!-- ===== 起点 / 终点标记 ===== -->
{marker_model('start_marker', spawn[0], spawn[1], 0.09, (0.1, 0.8, 0.2, 1))}
{marker_model('exit_marker', exit_world[0], exit_world[1], 0.12, (0.9, 0.1, 0.1, 1))}

    <!-- 默认观察视角：从南侧斜俯视整个迷宫。这样每次启动都能一眼看全，
         录演示视频时也不用先手动拖视角。
         注意：SDF 里 <gui> 必须是 <world> 的子元素，放到 </world> 外面不生效。 -->
    <gui fullscreen="0">
      <camera name="user_camera">
        <pose>{cx_maze:.2f} {cy_maze - 2.90:.2f} 3.10 0 0.66 1.5708</pose>
      </camera>
    </gui>

  </world>
</sdf>
""")
    return "\n".join(parts)


# ----------------------------------------------------------------------------
# 4. 主函数
# ----------------------------------------------------------------------------
def main(argv=None):
    ap = argparse.ArgumentParser(description='生成迷宫 Gazebo world')
    ap.add_argument('--cols', type=int, default=5)
    ap.add_argument('--rows', type=int, default=5)
    ap.add_argument('--cell', type=float, default=0.60, help='通道宽度 m')
    ap.add_argument('--thickness', type=float, default=0.06, help='墙厚 m')
    ap.add_argument('--height', type=float, default=0.30, help='墙高 m')
    ap.add_argument('--seed', type=int, default=2026)
    ap.add_argument('--out', default='maze.world')
    ap.add_argument('--json', default='', help='同时输出布局 json（供离线仿真使用）')
    ap.add_argument('--check', action='store_true', help='只做迷宫自检')
    args = ap.parse_args(argv)

    pitch = args.cell + args.thickness
    entry_row = 0
    exit_row = args.rows - 1

    # 入口格必须"正前方（东侧）通畅"，否则小车一出生就顶着墙，
    # 左手法则会立刻判定为死角。这里自动往后找一个满足条件的种子，
    # 保证任何人用任何 seed 生成出来的迷宫都能直接跑。
    seed = args.seed
    while True:
        v_wall, h_wall = generate_maze(args.cols, args.rows, seed)
        if not v_wall[1][entry_row]:
            break
        seed += 1
        if seed - args.seed > 500:
            print('[错误] 找不到入口正向通畅的种子', file=sys.stderr)
            return 1
    if seed != args.seed:
        print(f'[提示] 原种子 {args.seed} 的入口格正向是墙，已自动改用种子 {seed}')
    # 打通入口与出口
    v_wall[0][entry_row] = False
    v_wall[args.cols][exit_row] = False

    ok, reached, edges, total, path_ok = check_maze(
        v_wall, h_wall, args.cols, args.rows, entry_row, exit_row)

    print(f'[自检] 格子数 {total}, 连通 {reached}, 通路数 {edges} '
          f'(生成树应为 {total - 1}) -> {"PASS" if ok else "FAIL"}')
    print(f'[自检] 入口/出口可达 -> {"PASS" if path_ok else "FAIL"}')
    path = bfs_path(v_wall, h_wall, args.cols, args.rows, (0, entry_row),
                    (args.cols - 1, exit_row))
    print(f'[参考] 入口->出口最短格子路径长度 {len(path)} 步，'
          f'路径 {path}')

    if not (ok and path_ok):
        print('[错误] 迷宫不合格，请更换 --seed', file=sys.stderr)
        return 1
    if args.check:
        return 0

    boxes = (merge_vertical(v_wall, args.cols, args.rows, pitch, args.thickness)
             + merge_horizontal(h_wall, args.cols, args.rows, pitch, args.thickness))

    # ---- 出发区（起始栏）：在入口外再围一格，形成"死胡同式"起步区 ----
    # 为什么需要它：左手法则在死胡同里会掉头，若入口直接通向迷宫外，
    # 小车掉头时可能从入口钻出去，然后在空地上乱转（离线仿真真发生过）。
    # 围出这个 1 格长的出发区后，掉头开车回来只会撞到"西端墙"，
    # 于是被判定为死胡同 → 再次掉头回迷宫，物理上彻底堵死跑飞的可能。
    boxes += [
        (-pitch, pitch / 2.0, args.thickness, pitch + args.thickness),   # 西端墙
        (-pitch / 2.0, pitch, pitch, args.thickness),                    # 出发区北墙
        (-pitch / 2.0, 0.0, pitch, args.thickness),                      # 出发区南墙
    ]
    print(f'[生成] 墙体盒体 {len(boxes)} 个（已合并相邻墙，含出发区 3 面墙）')

    # 出生点 = 出发区中心，车头朝东（正对入口）
    spawn = (-pitch / 2.0, pitch / 2.0, 0.0)
    exit_world = (args.cols * pitch + 0.55, exit_row * pitch + pitch / 2.0)

    world = build_world(boxes, args.cols, args.rows, pitch, args.thickness,
                        args.height, spawn, exit_world, entry_row)
    with open(args.out, 'w', encoding='utf-8') as f:
        f.write(world)
    print(f'[生成] world 文件 -> {args.out}')

    if args.json:
        segs = []
        for cx, cy, sx, sy in boxes:
            segs.extend(box_to_segments(cx, cy, sx, sy))
        data = {
            'cols': args.cols, 'rows': args.rows,
            'cell': args.cell, 'thickness': args.thickness, 'pitch': pitch,
            'spawn': list(spawn),
            'exit_x_world': args.cols * pitch,
            'exit_row': exit_row,
            'boxes': boxes,
            'segments': segs,
            'v_wall': v_wall, 'h_wall': h_wall,
            'path': path,
        }
        with open(args.json, 'w', encoding='utf-8') as f:
            json.dump(data, f, ensure_ascii=False)
        print(f'[生成] 布局 json -> {args.json}')

    print(f'[提示] 出生点 x={spawn[0]:.2f} y={spawn[1]:.2f}；'
          f'出口墙位于世界系 x={args.cols * pitch:.2f}，'
          f'对应里程计 x={args.cols * pitch - spawn[0]:.2f}')
    return 0


if __name__ == '__main__':
    sys.exit(main())

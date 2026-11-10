#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""实时轨迹可视化：订阅 /odom，边跑边把车走过的位置连成轨迹。

为什么要单独做这个节点：
  仿真里只看 Gazebo 的 3D 画面，很难判断"这一趟到底走了什么形状"。
  本节点把 /odom 的位置点一个个记下来并实时画出来，跑完就是一张
  "迷宫 + 轨迹"的图，既方便调参对比，也方便放进报告和答辩 PPT。

用法：
    # 先生成布局 json（含墙体盒子，用来画迷宫背景；不生成也能画，只是没有墙）
    ros2 run maze_mouse gen_maze_world --out /tmp/maze.world --json /tmp/maze_layout.json

    # 再和算法节点一起跑
    ros2 run maze_mouse traj_plot
    ros2 run maze_mouse traj_plot --ros-args -p layout:=/tmp/maze_layout.json \
                                            -p save:=/tmp/trajectory.png
"""

import json
import math
import os

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

import matplotlib
matplotlib.use('TkAgg')          # 需要 python3-tk；没有窗口时改成 'Agg' 只存图
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

# 中文字体：直接把字体文件注册进 matplotlib，避免"家族名解析不到 → 变方块"
import matplotlib.font_manager as _fm

for _cjk in ('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc',):
    if os.path.exists(_cjk):
        try:
            _fm.fontManager.addfont(_cjk)
            plt.rcParams['font.sans-serif'] = [_fm.FontProperties(fname=_cjk).get_name()]
            break
        except Exception:
            pass
else:
    plt.rcParams['font.sans-serif'] = ['Noto Sans CJK SC', 'DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False


class TrajPlot(Node):
    """订阅 /odom，把位置点累积成轨迹并实时刷新。"""

    def __init__(self):
        super().__init__('traj_plot')

        self.declare_parameter('layout', '')        # 迷宫布局 json（可选）
        self.declare_parameter('min_step', 0.01)    # 相邻两点最小间距（m），更小会记更多点
        self.declare_parameter('save', '')          # 退出时另存 PNG（可选）

        self.layout_path = str(self.get_parameter('layout').value)
        self.min_step = float(self.get_parameter('min_step').value)
        self.save_path = str(self.get_parameter('save').value)

        # ---- 坐标换算：里程计原点在出生点，迷宫用的是世界坐标 ----
        self.spawn = (-0.33, 0.33)
        self.boxes = []
        self.exit_xy = None
        self._load_layout()

        self.xs, self.ys = [], []
        self.pose = None
        self._changed = False
        self._path_len = 0.0

        # ---- 画布 ----
        self.fig, self.ax = plt.subplots(figsize=(7.6, 7.6))
        self.fig.canvas.manager.set_window_title('traj_plot - /odom 累计轨迹')
        try:
            self.fig.canvas.manager.window.geometry('1180x760+40+10')
        except Exception:
            pass
        self._draw_background()

        (self.line,) = self.ax.plot([], [], '-', color='#1F6FEB', lw=2.0,
                                    label='轨迹（/odom）')
        (self.pts,) = self.ax.plot([], [], '.', color='#1F6FEB', ms=2.4, alpha=0.85)
        (self.cur,) = self.ax.plot([], [], 'o', color='#D1242F', ms=9, label='当前位置')
        self.head = None
        self.txt = self.ax.text(0.02, 0.98, '', transform=self.ax.transAxes,
                                va='top', ha='left', fontsize=10, color='#333333')

        self.sub = self.create_subscription(Odometry, 'odom', self.odom_cb, 20)
        self.get_logger().info(
            f'轨迹可视化启动 | 最小点距 {self.min_step * 100:.0f} cm | '
            f'布局 {"已加载" if self.boxes else "未加载（只画轨迹）"}')

    # ---------------------------------------------------------------- 背景
    def _load_layout(self):
        p = self.layout_path
        if not p or not os.path.exists(p):
            return
        try:
            with open(p, encoding='utf-8') as f:
                d = json.load(f)
            self.spawn = tuple(d.get('spawn', self.spawn)[:2])
            self.boxes = d.get('boxes', [])
            row = d.get('exit_row')
            if row is not None and d.get('pitch'):
                self.exit_xy = (d['exit_x_world'], 0.33 + row * d['pitch'])
        except Exception as e:      # 布局坏了也不能影响画轨迹
            self.get_logger().warn(f'布局文件读取失败：{e}')

    def _draw_background(self):
        ax = self.ax
        for cx, cy, sx, sy in self.boxes:
            ax.add_patch(Rectangle((cx - sx / 2, cy - sy / 2), sx, sy,
                                   facecolor='#B9C0C9', edgecolor='#8C959F',
                                   lw=0.6, zorder=1))
        ax.set_xlim(-0.9, 3.9)
        ax.set_ylim(-0.5, 3.7)
        ax.set_aspect('equal')
        ax.set_xlabel('x / m')
        ax.set_ylabel('y / m')
        ax.grid(True, ls=':', lw=0.5, color='#D8DEE4', zorder=0)
        ax.set_title('迷宫运动轨迹（点由 /odom 逐个累积）', fontsize=12)
        if self.exit_xy:
            ax.plot(*self.exit_xy, 'x', color='#CF222E', ms=12, mew=2.4,
                    label='出口', zorder=4)
        ax.plot(*self.spawn, 'o', color='#1A7F37', ms=8, label='出发点', zorder=4)

    # ---------------------------------------------------------------- 数据
    def odom_cb(self, msg: Odometry):
        x = msg.pose.pose.position.x + self.spawn[0]     # 里程计 → 世界坐标
        y = msg.pose.pose.position.y + self.spawn[1]
        q = msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))
        self.pose = (x, y, yaw)

        if self.xs:
            d = math.hypot(x - self.xs[-1], y - self.ys[-1])
            if d < self.min_step:
                return
            self._path_len += d
        self.xs.append(x)
        self.ys.append(y)
        self._changed = True

    # ---------------------------------------------------------------- 刷新
    def redraw(self):
        if not self._changed:
            return
        self._changed = False
        self.line.set_data(self.xs, self.ys)
        self.pts.set_data(self.xs, self.ys)
        if self.pose:
            x, y, yaw = self.pose
            self.cur.set_data([x], [y])
            if self.head is not None:
                self.head.remove()
            self.head = self.ax.annotate(
                '', xy=(x + 0.16 * math.cos(yaw), y + 0.16 * math.sin(yaw)),
                xytext=(x, y),
                arrowprops=dict(arrowstyle='-|>', color='#D1242F', lw=2.0),
                zorder=5)
            self.txt.set_text(
                f'点数 {len(self.xs)}　里程 {self._path_len:.2f} m\n'
                f'当前位置 ({x:+.2f}, {y:+.2f})')
        self.ax.legend(loc='lower right', fontsize=9, framealpha=0.9)
        self.fig.canvas.draw_idle()
        plt.pause(0.004)

    def finish(self):
        if self.save_path:
            self.fig.savefig(self.save_path, dpi=130, bbox_inches='tight')
            self.get_logger().info(f'轨迹图已保存：{self.save_path}')
        plt.close(self.fig)


def main(args=None):
    rclpy.init(args=args)
    node = TrajPlot()
    plt.show(block=False)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            node.redraw()
            if not plt.fignum_exists(node.fig.number):
                break
    except KeyboardInterrupt:
        pass
    finally:
        node.finish()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

# -*- coding: utf-8 -*-
"""
maze_mouse_core.py —— 纯 Python 算法核心（完全不依赖 ROS2）

为什么单独抽出一个"不依赖 ROS"的文件？
  1) 算法可以被单元测试 / 离线仿真直接调用，不需要启动 Gazebo；
  2) 参数整定时可以在几秒内跑上百次仿真，而 Gazebo 跑一次要几分钟；
  3) ROS2 节点只负责"收话题 -> 调这里 -> 发话题"，职责单一，报告里也容易讲清楚。

本文件提供四组能力：
  A. 角度 / 四元数等数学工具          norm_angle / clamp / quat_to_yaw
  B. 基于里程计的 P 点对点控制器        P2PController
  C. /scan 极坐标数据的分区取距离       sector_min / scan_zones
  D. 左墙跟随状态机                    CellWallFollower
"""

from __future__ import annotations

import math

TWO_PI = 2.0 * math.pi


# ----------------------------------------------------------------------------
# A. 数学工具
# ----------------------------------------------------------------------------
def norm_angle(angle: float) -> float:
    """把任意角度归一化到 [-pi, pi)。

    机器人控制里几乎所有"角度相减"都必须做这一步，否则会出现
    "目标在 +179°、当前 -179°，误算成 -358° 而原地打转"的经典 bug。
    """
    return (angle + math.pi) % TWO_PI - math.pi


def clamp(value: float, low: float, high: float) -> float:
    """把 value 限制在 [low, high] 内。"""
    return low if value < low else high if value > high else value


def quat_to_yaw(x: float, y: float, z: float, w: float) -> float:
    """四元数 -> 偏航角 yaw（绕 z 轴，逆时针为正，单位 rad）。

    /odom 与 /tf 给出的姿态都是四元数，控制律需要的是 yaw，转换公式：
        yaw = atan2(2(wz + xy), 1 - 2(y^2 + z^2))
    这样写可以避免引入 tf_transformations 依赖。
    """
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


# ----------------------------------------------------------------------------
# B. 点对点 P 控制器（基于里程计闭环）
# ----------------------------------------------------------------------------
class P2PController:
    """点到点闭环 P 控制器：先对准，再直行。

    控制思想（两段式，比"直接 v = Kp * dist"稳定得多）：
      1) 角度环：alpha = 目标方位角 - 当前 yaw，w = Kp_ang * alpha
      2) 位置环：v = Kp_lin * dist
      3) 当 |alpha| > ang_align 时先原地转（v = 0），把航向对准目标；
         对准后再边走边微调航向，形成"弧线逼近"的自然轨迹。
    """

    def __init__(self,
                 kp_lin: float = 0.8,       # 位置环比例系数 [1/s]
                 kp_ang: float = 1.8,       # 角度环比例系数 [1/s]
                 v_max: float = 0.20,       # 线速度上限 [m/s]
                 w_max: float = 1.20,       # 角速度上限 [rad/s]
                 v_min: float = 0.03,       # 最小线速度，避免最后几厘米"磨蹭不动"
                 tol_dist: float = 0.04,    # 到点判据：距离误差 [m]
                 tol_ang: float = 0.10,     # 到点判据：航向误差 [rad]
                 ang_align: float = 0.55):  # 超过该角度先原地转 [rad]
        self.kp_lin = kp_lin
        self.kp_ang = kp_ang
        self.v_max = v_max
        self.w_max = w_max
        self.v_min = v_min
        self.tol_dist = tol_dist
        self.tol_ang = tol_ang
        self.ang_align = ang_align

    def compute(self, pose, goal):
        """pose=(x, y, yaw)，goal=(gx, gy)。

        返回 (v, w, dist, alpha, reached)
          v, w    : 下发给 /cmd_vel 的速度指令
          dist    : 当前到目标的直线距离
          alpha   : 目标方位角与当前航向的偏差（已归一化）
          reached : 是否满足到点判据
        """
        x, y, yaw = pose
        gx, gy = goal

        dx = gx - x
        dy = gy - y
        dist = math.hypot(dx, dy)
        alpha = norm_angle(math.atan2(dy, dx) - yaw)

        w = clamp(self.kp_ang * alpha, -self.w_max, self.w_max)

        if abs(alpha) > self.ang_align:
            v = 0.0
        else:
            v = clamp(self.kp_lin * dist, 0.0, self.v_max)
            if 0.0 < v < self.v_min and dist > self.tol_dist:
                v = self.v_min

        reached = (dist < self.tol_dist) and (abs(alpha) < self.tol_ang)
        return v, w, dist, alpha, reached


# ----------------------------------------------------------------------------
# C. /scan 极坐标数据分区
# ----------------------------------------------------------------------------
def sector_min(ranges,
               angle_min: float,
               angle_increment: float,
               center: float,
               half_width: float,
               max_range: float = 3.5,
               min_valid: float = 0.02) -> float:
    """在一段扇形区域内取最小距离（即最近的墙）。

    参数
      ranges            : sensor_msgs/LaserScan.ranges，长度 360，索引 i 对应角度
      angle_min         : 起始角度（典型 -pi）
      angle_increment   : 相邻两点的角度间隔（典型 2pi/360）
      center / half_width : 扇区中心角与半宽（rad，逆时针为正）
      max_range         : 没有有效回波时返回的"无穷远"替代值

    注意：LaserScan 里 inf / nan 表示"该方向没有回波"（超量程或镜面反射），
    必须过滤掉，否则 min() 拿到 inf 会让所有判据失效。
    """
    best = max_range
    n = len(ranges)
    for i in range(n):
        r = ranges[i]
        if not math.isfinite(r) or r < min_valid:
            continue
        a = angle_min + i * angle_increment
        if abs(norm_angle(a - center)) <= half_width:
            if r < best:
                best = r
    return best


# 四个分区的中心角与半宽。
# 半宽只取 ±12°（很窄），这是被几何"逼"出来的：走廊只有 0.60 m 宽，
# 若把左侧扇区开到 ±30°，在路口处取最小值会把"对面那面墙的拐角"
# 算进来（实测读数 0.52 m，而真值是"左方开阔、距离 0.96 m"），
# 于是左手法则会把可走的路口误判成墙。窄扇区 + 格心采样才能拿到干净读数。
SECTOR_HALF = math.radians(12)
ZONE_DEF = {
    'front': (0.0, SECTOR_HALF),
    'left': (math.radians(90), SECTOR_HALF),
    'right': (-math.radians(90), SECTOR_HALF),
    'rear': (math.radians(180), SECTOR_HALF),
}


def scan_zones(ranges, angle_min: float, angle_increment: float,
               max_range: float = 3.5) -> dict:
    """把 360° 扫描点分成前/左/右/后四个扇区，各取最小距离。

    返回 dict: {'front': float, 'left': float, 'right': float, 'rear': float}
    单位 m，越大表示越开阔。
    """
    out = {}
    for name, (center, half) in ZONE_DEF.items():
        out[name] = sector_min(ranges, angle_min, angle_increment,
                               center, half, max_range=max_range)
    return out


# ----------------------------------------------------------------------------
# D. 左墙跟随状态机
# ----------------------------------------------------------------------------
class CellWallFollower:
    """左手法则（Left-Hand Rule）状态机 —— 按"格"决策，用里程计执行。

    为什么不做"连续反应式"的 w = Kp*(left - d_des)？
      在本任务的 0.66 m 格距、0.60 m 走廊里，连续 PD 会被"墙角"这一
      几何细节干扰：车一过路口，左方最近的点往往不是左墙，而是身后那面墙
      的拐角（实测 left 会从 0.30 跳到 0.43 又跳到 0.90），控制律跟着抖，
      轨迹像喝醉。这个坑值得写进报告的"问题分析"。

    本实现采用 MicroMouse 竞赛的经典做法 —— 把连续空间离散成格：
      1) 车每走到一个格心就停一下，用车载雷达量"左/前/右"三面有没有墙；
      2) 按左手法则挑一个方向（左 > 前 > 右 > 掉头）；
      3) 用里程计把车精确转到该朝向（90° 的整数倍），再直行一格（0.66 m）。
    这样每一步的几何都干净、可复现，状态机也容易画进报告。

    状态转移图（报告第 4 章要画的那张图）：

         ┌────────┐  稳定 settle_time  ┌────────┐
         │ DECIDE │<───────────────────┤ADVANCE │  到达下一个格心（里程计判据）
         │ 读三面墙│                    │前进一格 │
         │ 左手法则│                    └────────┘
         └───┬────┘                          ▲
             │ 算出目标朝向/目标格              │
             ▼                                 │
         ┌────────┐   |航向误差| < turn_tol      │
         │  TURN  │────────────────────────────┘
         │原地转向 │
         └────────┘
    """

    DECIDE = 'DECIDE'
    TURN = 'TURN'
    ADVANCE = 'ADVANCE'
    ARRIVED = 'ARRIVED'

    # 方向索引：0=东(+x) 1=北(+y) 2=西(-x) 3=南(-y)，逆时针
    DIRS = ((1, 0), (0, 1), (-1, 0), (0, -1))
    DIR_NAMES = ('E', 'N', 'W', 'S')
    DECISION_TEXT = {
        'left_open': '左侧有路口 -> 左转（左手法则优先）',
        'front_open': '前方通畅 -> 直行',
        'right_open': '前方堵、右侧通畅 -> 右转',
        'dead_end': '三面皆墙 -> 掉头',
    }

    def __init__(self,
                 pitch: float = 0.66,        # 格距（通道 0.60 + 墙厚 0.06）[m]
                 v_nom: float = 0.18,        # 格间直行速度 [m/s]
                 v_min: float = 0.05,        # 临近格心时的最小速度 [m/s]
                 kp_turn: float = 2.2,       # 转向角度环增益
                 w_turn_max: float = 1.00,   # 转向角速度上限 [rad/s]
                 kp_head: float = 2.0,       # 直行时航向保持增益
                 w_head_max: float = 0.80,   # 直行时角速度上限 [rad/s]
                 k_center: float = 0.80,     # 居中修正增益（两侧都有墙时生效）
                 wall_thresh: float = 0.55,  # 判断"该方向有墙"的距离阈值 [m]
                 turn_tol: float = 0.035,    # 转向到位判据 [rad]
                 arrive_tol: float = 0.02,   # 到格心判据 [m]
                 settle_time: float = 0.30,  # 停车后等待雷达稳定的时间 [s]
                 cell_timeout: float = 25.0):  # 单格超时保护 [s]
        self.pitch = pitch
        self.v_nom = v_nom
        self.v_min = v_min
        self.kp_turn = kp_turn
        self.w_turn_max = w_turn_max
        self.kp_head = kp_head
        self.w_head_max = w_head_max
        self.k_center = k_center
        self.wall_thresh = wall_thresh
        self.turn_tol = turn_tol
        self.arrive_tol = arrive_tol
        self.settle_time = settle_time
        self.cell_timeout = cell_timeout

        self.cell = (0, 0)        # 当前所在格（里程计原点即为起始格中心）
        self.dir_idx = 0          # 当前朝向索引，0 = 东
        self.state = self.DECIDE
        self.target_yaw = 0.0
        self.target_xy = (0.0, 0.0)
        self._settle = settle_time
        self._cell_time = 0.0
        self.last_decision = 'init'
        self.history = []         # [(格坐标, 朝向名, 决策原因)]
        self.turn_count = 0
        self.step_count = 0

    # -- 工具 -------------------------------------------------------------
    def _forward(self):
        return math.cos(self.target_yaw), math.sin(self.target_yaw)

    def _reset_cell_timer(self):
        self._cell_time = 0.0

    # -- 单步控制 ---------------------------------------------------------
    def step(self, pose, zones, dt: float = 0.05):
        """输入里程计位姿与三分区距离，输出 (v, w, state)。

        pose  = (x, y, yaw)，单位 m / rad，原点在出生点（= 起始格中心）
        zones = scan_zones() 的返回值，至少含 front / left / right
        dt    = 调用周期（默认 20 Hz -> 0.05 s）
        """
        if dt <= 1e-6:
            dt = 0.05
        self.step_count += 1
        x, y, yaw = pose

        if self.state == self.ARRIVED:
            return 0.0, 0.0, self.ARRIVED

        # ================= DECIDE：停在格心，读三面墙，做左手法则决策 =======
        if self.state == self.DECIDE:
            self._settle -= dt
            if self._settle > 0.0:
                return 0.0, 0.0, self.DECIDE

            left_open = zones['left'] > self.wall_thresh
            front_open = zones['front'] > self.wall_thresh
            right_open = zones['right'] > self.wall_thresh

            if left_open:                      # 左手法则：能左转就左转
                new_dir = (self.dir_idx + 1) % 4
                self.last_decision = 'left_open'
            elif front_open:                   # 其次直行
                new_dir = self.dir_idx
                self.last_decision = 'front_open'
            elif right_open:                   # 再其次右转
                new_dir = (self.dir_idx + 3) % 4
                self.last_decision = 'right_open'
            else:                              # 三面皆墙：掉头
                new_dir = (self.dir_idx + 2) % 4
                self.last_decision = 'dead_end'

            dx, dy = self.DIRS[new_dir]
            self.cell = (self.cell[0] + dx, self.cell[1] + dy)
            self.target_xy = (self.cell[0] * self.pitch, self.cell[1] * self.pitch)
            self.dir_idx = new_dir
            self.target_yaw = new_dir * (math.pi / 2.0)
            self.history.append((self.cell, self.DIR_NAMES[new_dir], self.last_decision))
            self._reset_cell_timer()
            self.state = self.TURN
            return 0.0, 0.0, self.TURN

        # ================= TURN：用里程计角闭环转到目标朝向 ==================
        if self.state == self.TURN:
            self._cell_time += dt
            err = norm_angle(self.target_yaw - yaw)
            if abs(err) < self.turn_tol or self._cell_time > self.cell_timeout:
                self.state = self.ADVANCE
                return 0.0, 0.0, self.ADVANCE
            w = clamp(self.kp_turn * err, -self.w_turn_max, self.w_turn_max)
            return 0.0, w, self.TURN

        # ================= ADVANCE：直行一格 ================================
        self._cell_time += dt
        tx, ty = self.target_xy
        dx, dy = tx - x, ty - y
        dist = math.hypot(dx, dy)
        fx, fy = self._forward()
        passed = (dx * fx + dy * fy) < 0.0        # 已经越过目标格心

        if dist < self.arrive_tol or (passed and dist < 0.15):
            self._settle = self.settle_time
            self.state = self.DECIDE
            return 0.0, 0.0, self.DECIDE

        if self._cell_time > self.cell_timeout:   # 卡住了，强制重新决策
            self._settle = self.settle_time
            self.state = self.DECIDE
            return 0.0, 0.0, self.DECIDE

        # 航向保持 + 居中修正（两侧都是墙时才有意义的对称修正）
        w = clamp(self.kp_head * norm_angle(self.target_yaw - yaw),
                  -self.w_head_max, self.w_head_max)
        if zones['left'] < self.wall_thresh and zones['right'] < self.wall_thresh:
            w = clamp(w + self.k_center * (zones['left'] - zones['right']),
                      -self.w_head_max, self.w_head_max)

        # 临近目标减速，保证停在格心附近而不是冲过头
        v = min(self.v_nom, max(self.v_min, 2.0 * dist))
        return v, w, self.ADVANCE

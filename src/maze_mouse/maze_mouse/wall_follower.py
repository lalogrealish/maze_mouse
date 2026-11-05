#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""迷宫左墙跟随主控节点（感知 + 决策 + 控制 三合一）。

订阅：/scan  sensor_msgs/LaserScan   -> 前/左/右分区距离
订阅：/odom  nav_msgs/Odometry       -> 位姿、航向、出迷宫判定
发布：/cmd_vel      geometry_msgs/Twist   -> 速度指令
发布：/wall_state   std_msgs/String       -> 当前状态（录视频讲解时很有用）

算法核心在 maze_mouse_core.CellWallFollower（纯 Python，可离线单测），
本节点只负责 ROS2 通信和"出迷宫判定"。

用法：
    ros2 run maze_mouse wall_follower
    ros2 run maze_mouse wall_follower --ros-args -p exit_x:=3.75 -p v_nom:=0.22
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from maze_mouse.maze_mouse_core import CellWallFollower, scan_zones, quat_to_yaw


class WallFollower(Node):
    def __init__(self):
        super().__init__('wall_follower')

        # ---- 出迷宫判定 ----
        # 里程计用编码器积分（URDF 里 odometry_source=0），原点就是出生点。
        # 出生点世界系 x=-0.33，出口门洞世界系 x=3.30 -> 出口在 odom 下是 3.63；
        # 默认 3.75，让车完整驶出洞口约 12 cm 再停车。
        self.declare_parameter('exit_x', 3.75)
        self.declare_parameter('use_exit_detection', True)

        # ---- 控制器参数（全部可调，便于做对比实验） ----
        self.declare_parameter('pitch', 0.66)         # 格距
        self.declare_parameter('v_nom', 0.18)         # 格间直行速度
        self.declare_parameter('v_min', 0.05)
        self.declare_parameter('kp_turn', 2.2)
        self.declare_parameter('w_turn_max', 1.00)
        self.declare_parameter('kp_head', 2.0)
        self.declare_parameter('w_head_max', 0.80)
        self.declare_parameter('k_center', 0.80)
        self.declare_parameter('wall_thresh', 0.55)   # 判断"有墙"的距离阈值
        self.declare_parameter('turn_tol', 0.035)
        self.declare_parameter('arrive_tol', 0.02)
        self.declare_parameter('settle_time', 0.30)
        self.declare_parameter('control_rate', 20.0)

        p = self.get_parameter
        self.ctrl = CellWallFollower(
            pitch=float(p('pitch').value),
            v_nom=float(p('v_nom').value),
            v_min=float(p('v_min').value),
            kp_turn=float(p('kp_turn').value),
            w_turn_max=float(p('w_turn_max').value),
            kp_head=float(p('kp_head').value),
            w_head_max=float(p('w_head_max').value),
            k_center=float(p('k_center').value),
            wall_thresh=float(p('wall_thresh').value),
            turn_tol=float(p('turn_tol').value),
            arrive_tol=float(p('arrive_tol').value),
            settle_time=float(p('settle_time').value))

        self.exit_x = float(p('exit_x').value)
        self.use_exit = bool(p('use_exit_detection').value)
        rate = float(p('control_rate').value)
        self.dt = 1.0 / rate

        self.zones = None
        self.pose = None
        self.state_name = CellWallFollower.DECIDE
        self.arrived = False
        self.step_count = 0
        self.turn_count = 0

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.state_pub = self.create_publisher(String, 'wall_state', 10)
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, qos_profile_sensor_data)
        self.odom_sub = self.create_subscription(
            Odometry, 'odom', self.odom_callback, 10)
        self.timer = self.create_timer(self.dt, self.control_loop)

        self.get_logger().info(
            f'左墙跟随启动 | 直行速度 {self.ctrl.v_nom:.2f} m/s | '
            f'墙判据 {self.ctrl.wall_thresh:.2f} m | '
            f'出迷宫判定 odom.x > {self.exit_x:.2f} m')

    def scan_callback(self, msg: LaserScan):
        self.zones = scan_zones(msg.ranges, msg.angle_min, msg.angle_increment,
                                max_range=msg.range_max)

    def odom_callback(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.pose = (p.x, p.y, quat_to_yaw(q.x, q.y, q.z, q.w))

    def _publish_state(self):
        msg = String()
        msg.data = self.state_name
        self.state_pub.publish(msg)

    def _stop(self, reason: str):
        self.arrived = True
        self.cmd_pub.publish(Twist())
        self.state_name = CellWallFollower.ARRIVED
        self._publish_state()
        self.timer.cancel()
        self.get_logger().info(f'==== 到达终点：{reason} ====')
        if self.pose is not None:
            self.get_logger().info(
                f'终点里程计位姿 x={self.pose[0]:.2f} y={self.pose[1]:.2f} '
                f'yaw={self.pose[2]:.2f} rad，共决策 {len(self.ctrl.history)} 格')

    def control_loop(self):
        if self.zones is None or self.pose is None or self.arrived:
            return

        # 1) 出迷宫判定放最前面：一出洞口立刻停车，避免冲出地面
        if self.use_exit and self.pose[0] > self.exit_x:
            self._stop(f'里程计 x={self.pose[0]:.2f} 已越过出口门限 {self.exit_x:.2f}')
            return

        # 2) 状态机 + 控制
        v, w, state = self.ctrl.step(self.pose, self.zones, self.dt)

        if state != self.state_name:
            if state == CellWallFollower.TURN:
                self.turn_count += 1
                self.get_logger().info(
                    f'决策[{self.ctrl.last_decision}] 格{self.ctrl.cell} -> '
                    f'目标航向 {self.ctrl.target_yaw:+.2f} rad')
            elif state == CellWallFollower.ADVANCE:
                self.get_logger().info(
                    f'开始直行 -> 目标 ({self.ctrl.target_xy[0]:+.2f}, '
                    f'{self.ctrl.target_xy[1]:+.2f})')
            self.state_name = state
            self._publish_state()

        msg = Twist()
        msg.linear.x = v
        msg.angular.z = w
        self.cmd_pub.publish(msg)

        self.step_count += 1
        if self.step_count % 40 == 0:      # 2 s 打印一次
            z = self.zones
            self.get_logger().info(
                f'[{state}] pos=({self.pose[0]:+.2f},{self.pose[1]:+.2f}) '
                f'yaw={self.pose[2]:+.2f} 前 {z["front"]:.2f} '
                f'左 {z["left"]:.2f} 右 {z["right"]:.2f} -> v={v:.2f} w={w:+.2f}')


def main(args=None):
    rclpy.init(args=args)
    node = WallFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

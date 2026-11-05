#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""点到点闭环 P 控制节点：靠 /odom 反馈把小车开到 (goal_x, goal_y)。

这是"里程计运动闭环"的核心实验：
  开环：只管发指令，不管结果       -> 误差累积、走不准
  闭环：measure -> compare -> correct -> 误差收敛

用法：
    # 从出生点开到 (2.0, 0.5)
    ros2 run maze_mouse p2p_control --ros-args -p goal_x:=2.0 -p goal_y:=0.5
    # 调参实验：把 kp_lin 从 0.4 加到 1.2，观察超调
    ros2 run maze_mouse p2p_control --ros-args -p goal_x:=2.0 -p goal_y:=0.5 -p kp_lin:=1.2
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from maze_mouse.maze_mouse_core import P2PController, quat_to_yaw


class P2PControl(Node):
    def __init__(self):
        super().__init__('p2p_control')

        self.declare_parameter('goal_x', 1.0)
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('kp_lin', 0.8)
        self.declare_parameter('kp_ang', 1.8)
        self.declare_parameter('v_max', 0.20)
        self.declare_parameter('w_max', 1.20)
        self.declare_parameter('tol_dist', 0.04)
        self.declare_parameter('tol_ang', 0.10)
        self.declare_parameter('control_rate', 20.0)

        self.goal = (float(self.get_parameter('goal_x').value),
                     float(self.get_parameter('goal_y').value))

        self.ctrl = P2PController(
            kp_lin=float(self.get_parameter('kp_lin').value),
            kp_ang=float(self.get_parameter('kp_ang').value),
            v_max=float(self.get_parameter('v_max').value),
            w_max=float(self.get_parameter('w_max').value),
            tol_dist=float(self.get_parameter('tol_dist').value),
            tol_ang=float(self.get_parameter('tol_ang').value))

        self.pose = None
        self.done = False
        self.last_log = self.get_clock().now()

        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)

        rate = float(self.get_parameter('control_rate').value)
        self.timer = self.create_timer(1.0 / rate, self.control_loop)

        self.get_logger().info(
            f'P 控制启动，目标点 ({self.goal[0]:.2f}, {self.goal[1]:.2f})，'
            f'kp_lin={self.ctrl.kp_lin}, kp_ang={self.ctrl.kp_ang}')

    def odom_callback(self, msg: Odometry):
        """里程计回调：只做一件事 —— 把四元数姿态转成 yaw 存起来。"""
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.pose = (p.x, p.y, quat_to_yaw(q.x, q.y, q.z, q.w))

    def control_loop(self):
        if self.pose is None or self.done:
            return

        v, w, dist, alpha, reached = self.ctrl.compute(self.pose, self.goal)

        if reached:
            self.pub.publish(Twist())      # 到点刹车
            self.done = True
            self.timer.cancel()
            self.get_logger().info(
                f'已到达目标点，终点误差 {dist * 1000:.1f} mm，'
                f'航向误差 {math.degrees(alpha):.1f}°')
            return

        msg = Twist()
        msg.linear.x = v
        msg.angular.z = w
        self.pub.publish(msg)

        # 每 0.5 s 打印一次，方便录视频时讲解
        if (self.get_clock().now() - self.last_log).nanoseconds > 5e8:
            self.last_log = self.get_clock().now()
            self.get_logger().info(
                f'位置 ({self.pose[0]:.2f}, {self.pose[1]:.2f}) '
                f'距目标 {dist:.3f} m  航向差 {math.degrees(alpha):+.1f}°  '
                f'v={v:.3f} w={w:+.3f}')


def main(args=None):
    rclpy.init(args=args)
    node = P2PControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

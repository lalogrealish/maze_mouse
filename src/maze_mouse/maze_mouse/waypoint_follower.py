#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""航点序列跟踪节点：依次经过多个坐标点。

与 p2p_control 的区别：这里维护一个"航点队列 + 当前索引"，
到达当前点后自动切换下一个，直到全部走完。这是路径跟踪的最小雏形，
拓展任务里的 Pure-Pursuit 本质上就是"把离散航点换成连续参考路径"。

用法：
    ros2 run maze_mouse waypoint_follower
    # 自定义 3 个航点：(1,0) -> (1,1) -> (0,1)
    ros2 run maze_mouse waypoint_follower --ros-args -p waypoints:="[1.0,0.0,1.0,1.0,0.0,1.0]"
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from maze_mouse.maze_mouse_core import P2PController, quat_to_yaw


class WaypointFollower(Node):
    def __init__(self):
        super().__init__('waypoint_follower')

        # 默认航线：在迷宫外侧的空地上绕一个矩形，不会撞墙
        default_wp = [-0.80, 1.20,
                      -2.00, 1.20,
                      -2.00, -0.60,
                      -0.80, -0.60]
        self.declare_parameter('waypoints', default_wp)
        self.declare_parameter('kp_lin', 0.8)
        self.declare_parameter('kp_ang', 1.8)
        self.declare_parameter('v_max', 0.20)
        self.declare_parameter('tol_dist', 0.06)
        self.declare_parameter('control_rate', 20.0)
        self.declare_parameter('loop', False)   # True = 走完一遍再从头开始

        flat = list(self.get_parameter('waypoints').value)
        if len(flat) % 2 != 0:
            self.get_logger().error('waypoints 必须是偶数个元素 [x1,y1,x2,y2,...]')
            flat = flat[:-1]
        self.waypoints = [(float(flat[i]), float(flat[i + 1]))
                          for i in range(0, len(flat), 2)]

        self.ctrl = P2PController(
            kp_lin=float(self.get_parameter('kp_lin').value),
            kp_ang=float(self.get_parameter('kp_ang').value),
            v_max=float(self.get_parameter('v_max').value),
            tol_dist=float(self.get_parameter('tol_dist').value),
            # 航点跟踪允许更宽松的航向误差，否则每个点都要"停—转—走"
            tol_ang=0.35)

        self.loop = bool(self.get_parameter('loop').value)
        self.idx = 0
        self.pose = None
        self.done = False

        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        rate = float(self.get_parameter('control_rate').value)
        self.timer = self.create_timer(1.0 / rate, self.control_loop)

        self.get_logger().info(f'共 {len(self.waypoints)} 个航点：{self.waypoints}')
        if self.waypoints:
            self.get_logger().info(f'前往第 1 个航点 {self.waypoints[0]}')

    def odom_callback(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.pose = (p.x, p.y, quat_to_yaw(q.x, q.y, q.z, q.w))

    def control_loop(self):
        if self.pose is None or self.done or not self.waypoints:
            return

        target = self.waypoints[self.idx]
        v, w, dist, alpha, reached = self.ctrl.compute(self.pose, target)

        if reached:
            self.idx += 1
            if self.idx >= len(self.waypoints):
                if self.loop:
                    self.idx = 0
                    self.get_logger().info('一圈完成，重新从第 1 个航点开始')
                    return
                self.pub.publish(Twist())
                self.done = True
                self.timer.cancel()
                self.get_logger().info(f'全部 {len(self.waypoints)} 个航点跟踪完成')
                return
            self.get_logger().info(
                f'✔ 到达航点 {self.idx}/{len(self.waypoints)}，'
                f'切换目标 -> {self.waypoints[self.idx]}')
            return

        msg = Twist()
        msg.linear.x = v
        msg.angular.z = w
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollower()
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

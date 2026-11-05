#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""反应式避障节点：前方有障碍就减速/转向，是最基础的"感知→动作"闭环。

订阅：/scan   sensor_msgs/LaserScan
发布：/cmd_vel   geometry_msgs/Twist
发布：/avoid_state  std_msgs/String   （FORWARD / SLOW / TURN / STOP）

控制逻辑只有三层，简单但足够演示"反应式"三个字：
    1) front > d_slow          -> 全速直行
    2) d_stop < front <= d_slow-> 按距离线性减速，同时向更开阔的一侧微调航向
    3) front <= d_stop         -> 原地转向更开阔的一侧（也可换成直接停车）

用法：
    ros2 run maze_mouse reactive_avoid
    ros2 run maze_mouse reactive_avoid --ros-args -p mode:=stop      # 只停不转
    ros2 run maze_mouse reactive_avoid --ros-args -p d_stop:=0.6
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from maze_mouse.maze_mouse_core import clamp, scan_zones


class ReactiveAvoid(Node):
    def __init__(self):
        super().__init__('reactive_avoid')

        self.declare_parameter('v_norm', 0.15)      # 正常巡航速度 [m/s]
        self.declare_parameter('d_stop', 0.45)      # 小于该距离必须处理 [m]
        self.declare_parameter('d_slow', 0.90)      # 开始减速的距离 [m]
        self.declare_parameter('w_turn', 0.60)      # 避障转向角速度 [rad/s]
        self.declare_parameter('k_side', 0.6)       # 沿更开阔一侧微调的增益
        self.declare_parameter('mode', 'turn')      # turn = 转向绕开；stop = 直接停
        self.declare_parameter('control_rate', 20.0)

        p = self.get_parameter
        self.v_norm = float(p('v_norm').value)
        self.d_stop = float(p('d_stop').value)
        self.d_slow = float(p('d_slow').value)
        self.w_turn = float(p('w_turn').value)
        self.k_side = float(p('k_side').value)
        self.mode = str(p('mode').value).strip()

        self.zones = None
        self.state = 'INIT'
        self.count = 0

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.state_pub = self.create_publisher(String, 'avoid_state', 10)
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, qos_profile_sensor_data)
        self.timer = self.create_timer(1.0 / float(p('control_rate').value),
                                       self.control_loop)
        self.get_logger().info(
            f'反应式避障启动 | mode={self.mode} | '
            f'd_stop={self.d_stop:.2f} m | d_slow={self.d_slow:.2f} m')

    def scan_callback(self, msg: LaserScan):
        self.zones = scan_zones(msg.ranges, msg.angle_min, msg.angle_increment,
                                max_range=msg.range_max)

    def control_loop(self):
        if self.zones is None:
            return
        front = self.zones['front']
        left = self.zones['left']
        right = self.zones['right']

        msg = Twist()

        if front <= self.d_stop:
            if self.mode == 'stop':
                msg.linear.x = 0.0
                msg.angular.z = 0.0
                state = 'STOP'
            else:
                # 向更开阔的一侧转：左边远就往左（+z），右边远就往右（-z）
                msg.linear.x = 0.02
                msg.angular.z = self.w_turn if left >= right else -self.w_turn
                state = 'TURN'
        elif front < self.d_slow:
            # 线速度随剩余距离线性衰减：front=d_slow 时全速，front=d_stop 时为 0
            ratio = (front - self.d_stop) / (self.d_slow - self.d_stop)
            msg.linear.x = self.v_norm * clamp(ratio, 0.0, 1.0)
            msg.angular.z = clamp(self.k_side * (left - right),
                                  -self.w_turn, self.w_turn)
            state = 'SLOW'
        else:
            msg.linear.x = self.v_norm
            msg.angular.z = clamp(self.k_side * (left - right) * 0.3,
                                  -self.w_turn, self.w_turn)
            state = 'FORWARD'

        self.cmd_pub.publish(msg)

        if state != self.state:
            self.get_logger().info(
                f'状态 {self.state} -> {state} | 前 {front:.2f} 左 {left:.2f} 右 {right:.2f}')
            self.state = state
            s = String()
            s.data = state
            self.state_pub.publish(s)

        self.count += 1
        if self.count % 40 == 0:
            self.get_logger().info(
                f'[{state}] 前 {front:.2f} 左 {left:.2f} 右 {right:.2f} '
                f'-> v={msg.linear.x:.2f} w={msg.angular.z:+.2f}')


def main(args=None):
    rclpy.init(args=args)
    node = ReactiveAvoid()
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

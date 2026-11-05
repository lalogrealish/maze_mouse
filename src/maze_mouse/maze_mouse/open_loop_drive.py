#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""开环运动控制节点：定时发固定速度，不做任何反馈。

用途：验证"命令行能不能驱动小车"，是学习 ROS2 话题机制的第一个实验。

用法（三个实验分别跑）：
    # 1) 直线行驶 2 秒
    ros2 run maze_mouse open_loop_drive --ros-args -p mode:=forward -p value:=0.15 -p duration:=2.0
    # 2) 原地左转 90°（w=0.5 rad/s 约 3.14 s 转 90°）
    ros2 run maze_mouse open_loop_drive --ros-args -p mode:=turn -p value:=0.5 -p duration:=3.14

开环的"缺陷"正是实验要点：同样的 w 持续 3.14 s，电池电压、地面摩擦、
轮子打滑都会让实际转角偏离 90°，这就是下一步必须引入里程计闭环的原因。
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class OpenLoopDrive(Node):
    def __init__(self):
        super().__init__('open_loop_drive')

        # 参数：mode = forward（直行）/turn（原地转）/backward（后退）
        self.declare_parameter('mode', 'forward')
        self.declare_parameter('value', 0.15)      # 直行 m/s；转向 rad/s
        self.declare_parameter('duration', 2.0)    # 持续时间 s
        self.declare_parameter('rate', 20.0)       # 发布频率 Hz

        self.mode = self.get_parameter('mode').value
        self.value = float(self.get_parameter('value').value)
        self.duration = float(self.get_parameter('duration').value)
        rate = float(self.get_parameter('rate').value)

        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.msg = Twist()
        if self.mode == 'forward':
            self.msg.linear.x = abs(self.value)
        elif self.mode == 'backward':
            self.msg.linear.x = -abs(self.value)
        elif self.mode == 'turn':
            self.msg.angular.z = self.value
        else:
            self.get_logger().error(f'未知 mode={self.mode}，可选 forward/turn/backward')
            self.msg.linear.x = 0.0

        self.start_time = self.get_clock().now()
        self.finished = False
        self.timer = self.create_timer(1.0 / rate, self.timer_callback)
        self.get_logger().info(
            f'开环控制启动: mode={self.mode}, value={self.value}, duration={self.duration}s')

    def timer_callback(self):
        if self.finished:
            return
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9
        if elapsed >= self.duration:
            self.stop()
            return
        self.pub.publish(self.msg)

    def stop(self):
        """结束时必须发一条全零 Twist，否则小车会一直按最后一条指令动。"""
        self.finished = True
        self.pub.publish(Twist())
        self.get_logger().info(f'开环运动结束（实际耗时 {self.duration:.2f}s），已发送停车指令')
        self.timer.cancel()


def main(args=None):
    rclpy.init(args=args)
    node = OpenLoopDrive()
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

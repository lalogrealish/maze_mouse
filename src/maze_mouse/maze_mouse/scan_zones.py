#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""激光雷达分区提取节点：订阅 /scan，输出前/左/右/后四个方向的最小距离。

订阅：/scan            sensor_msgs/LaserScan
发布：/scan_zones      std_msgs/Float32MultiArray，数据顺序 [front, left, right, rear]

这个节点是"感知"环节的独立实验：先用它单独验证分区算得对不对，
再让左墙跟随节点复用同一套逻辑。分模块调试是本课程的重要方法。

用法：
    ros2 run maze_mouse scan_zones                       # 终端看表格
    ros2 run maze_mouse scan_zones -p zone_half_deg:=45.0
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray

from maze_mouse.maze_mouse_core import scan_zones


class ScanZones(Node):
    def __init__(self):
        super().__init__('scan_zones')

        self.declare_parameter('zone_half_deg', 35.0)   # 每个扇区的半宽（度）
        self.declare_parameter('print_period', 0.5)     # 终端打印周期 s

        self.pub = self.create_publisher(Float32MultiArray, 'scan_zones', 10)

        # 注意：雷达话题几乎都使用 BEST_EFFORT QoS，订阅端必须保持一致，
        # 否则会出现"话题能看到、就是收不到数据"的经典问题。
        self.sub = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, qos_profile_sensor_data)

        self.last_print = self.get_clock().now()
        self.print_period = float(self.get_parameter('print_period').value)
        self.get_logger().info('等待 /scan 数据 ...（若一直无输出，先检查仿真是否启动）')

    def scan_callback(self, msg: LaserScan):
        zones = scan_zones(msg.ranges, msg.angle_min, msg.angle_increment,
                           max_range=msg.range_max)

        out = Float32MultiArray()
        out.data = [float(zones['front']), float(zones['left']),
                    float(zones['right']), float(zones['rear'])]
        self.pub.publish(out)

        now = self.get_clock().now()
        if (now - self.last_print).nanoseconds > int(self.print_period * 1e9):
            self.last_print = now
            self.get_logger().info(
                f'前方 {zones["front"]:.2f} m | 左侧 {zones["left"]:.2f} m | '
                f'右侧 {zones["right"]:.2f} m | 后方 {zones["rear"]:.2f} m '
                f'(点数 {len(msg.ranges)}, 角度 {math.degrees(msg.angle_min):.0f}°'
                f'~{math.degrees(msg.angle_max):.0f}°)')


def main(args=None):
    rclpy.init(args=args)
    node = ScanZones()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

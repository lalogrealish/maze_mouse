#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""一键启动：Gazebo 世界 + 机器人模型 + RViz2 + 算法节点。

为什么必须用 launch 文件？
  手动开 5 个终端依次启动 gzserver / gzclient / robot_state_publisher /
  spawn_entity / rviz2，任何一步顺序错了（比如 spawn 早于 gzserver 就绪）
  都会失败，而且参数无法复现。launch 文件把这些固化成可版本管理的代码。

本文件做的事（也是报告第 3 章要讲的"节点调用关系"）：
  ┌─ gazebo.launch.py(Gazebo 官方) ── gzserver + gzclient，加载 world
  ├─ robot_state_publisher ────────── 解析 URDF，发 /robot_description 与 TF
  ├─ spawn_entity.py ──────────────── 把机器人从 /robot_description 插进 Gazebo
  ├─ rviz2 ────────────────────────── 可视化（可选）
  └─ 算法节点（可选）──────────────── wall_follower / waypoint_follower / p2p_control

用法：
    ros2 launch maze_mouse maze_bringup.launch.py
    ros2 launch maze_mouse maze_bringup.launch.py mission:=wall_follower
    ros2 launch maze_mouse maze_bringup.launch.py use_rviz:=false
    ros2 launch maze_mouse maze_bringup.launch.py world:=$HOME/maze_ws/src/maze_mouse/world/maze.world
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('maze_mouse')

    # ---------- 资源路径（全部来自 install/share，而不是 src） ----------
    world_path = os.path.join(pkg_share, 'world', 'maze.world')
    urdf_path = os.path.join(pkg_share, 'urdf', 'maze_mouse.urdf')
    rviz_cfg = os.path.join(pkg_share, 'config', 'maze.rviz')

    # URDF 一次性读成字符串，作为 robot_state_publisher 的参数
    with open(urdf_path, 'r', encoding='utf-8') as f:
        robot_description = f.read()

    # ---------- 可调参数 ----------
    declared = [
        DeclareLaunchArgument('world', default_value=world_path,
                              description='Gazebo world 文件绝对路径'),
        DeclareLaunchArgument('x', default_value='-0.33',
                              description='出生点 x（出发区中心，位于迷宫入口外一格）'),
        DeclareLaunchArgument('y', default_value='0.33',
                              description='出生点 y（对准入口通道中心）'),
        DeclareLaunchArgument('yaw', default_value='0.0',
                              description='出生点朝向 rad'),
        DeclareLaunchArgument('use_rviz', default_value='true',
                              description='是否启动 RViz2'),
        DeclareLaunchArgument('mission', default_value='none',
                              description='随仿真启动的算法节点: none|wall_follower|waypoint|p2p|avoid|zones'),
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='使用 Gazebo 仿真时钟'),
    ]

    use_sim_time = LaunchConfiguration('use_sim_time')

    # ---------- 1. Gazebo ----------
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'),
                         'launch', 'gazebo.launch.py')),
        launch_arguments={'world': LaunchConfiguration('world')}.items(),
    )

    # ---------- 2. 机器人状态发布 ----------
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description,
                     'use_sim_time': use_sim_time}],
    )

    # ---------- 3. 把机器人放进 Gazebo ----------
    # 用 -file 直接给文件：不依赖 /robot_description 的 QoS 时序，最省事。
    # 注意：URDF 的第一行不能带 encoding="utf-8"，否则 spawn_entity.py
    # 用 lxml 解析字符串时会抛 ValueError（本项目实际踩过的坑）。
    # -z 给 0.02 m 初始高度，让轮子从空中落下后与地面形成稳定接触。
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_maze_mouse',
        output='screen',
        arguments=['-file', urdf_path,
                   '-entity', 'maze_mouse',
                   '-x', LaunchConfiguration('x'),
                   '-y', LaunchConfiguration('y'),
                   '-z', '0.005',
                   '-Y', LaunchConfiguration('yaw')],
    )

    # ---------- 4. RViz2 ----------
    # 若用户已按指导文档保存过 config/maze.rviz，就带着配置启动；
    # 否则启动一个空白 RViz，由用户自己添加显示项（这也是实验要求之一）。
    rviz_args = ['-d', rviz_cfg] if os.path.exists(rviz_cfg) else []
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=rviz_args,
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

    # ---------- 5. 算法节点（按 mission 参数择一启动） ----------
    wall_follower = Node(
        package='maze_mouse', executable='wall_follower', name='wall_follower',
        output='screen', parameters=[{'use_sim_time': use_sim_time}],
        condition=LaunchConfigurationEquals('mission', 'wall_follower'),
    )
    waypoint_follower = Node(
        package='maze_mouse', executable='waypoint_follower', name='waypoint_follower',
        output='screen', parameters=[{'use_sim_time': use_sim_time}],
        condition=LaunchConfigurationEquals('mission', 'waypoint'),
    )
    p2p = Node(
        package='maze_mouse', executable='p2p_control', name='p2p_control',
        output='screen', parameters=[{'use_sim_time': use_sim_time}],
        condition=LaunchConfigurationEquals('mission', 'p2p'),
    )
    avoid = Node(
        package='maze_mouse', executable='reactive_avoid', name='reactive_avoid',
        output='screen', parameters=[{'use_sim_time': use_sim_time}],
        condition=LaunchConfigurationEquals('mission', 'avoid'),
    )
    zones = Node(
        package='maze_mouse', executable='scan_zones', name='scan_zones',
        output='screen', parameters=[{'use_sim_time': use_sim_time}],
        condition=LaunchConfigurationEquals('mission', 'zones'),
    )

    return LaunchDescription(declared + [
        gazebo,
        robot_state_publisher,
        spawn_entity,
        rviz2,
        wall_follower,
        waypoint_follower,
        p2p,
        avoid,
        zones,
    ])

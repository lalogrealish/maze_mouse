from glob import glob
import os

from setuptools import setup

package_name = 'maze_mouse'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        # 1) 让 ros2 run / ros2 launch 能找到这个包
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # 2) 把 package.xml 装进 share
        ('share/' + package_name, ['package.xml']),
        # 3) 资源文件：launch / urdf / world / config / rviz
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'world'), glob('world/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='student',
    maintainer_email='student@example.com',
    description='地老鼠走迷宫系统（ROS2 Humble + Gazebo Classic）',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'open_loop_drive = maze_mouse.open_loop_drive:main',
            'p2p_control = maze_mouse.p2p_control:main',
            'waypoint_follower = maze_mouse.waypoint_follower:main',
            'scan_zones = maze_mouse.scan_zones:main',
            'reactive_avoid = maze_mouse.reactive_avoid:main',
            'wall_follower = maze_mouse.wall_follower:main',
            'gen_maze_world = maze_mouse.gen_maze_world:main',
        ],
    },
)

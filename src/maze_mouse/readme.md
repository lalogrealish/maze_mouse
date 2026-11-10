# maze_mouse —— 地老鼠走迷宫系统

ROS2 Humble + Gazebo Classic 11 实现的地老鼠（MicroMouse）走迷宫系统：自建 TurtleBot3-Burger 尺寸 URDF 模型、自定义矩形墙体迷宫、Python launch 一键启动、里程计 P 控制、激光雷达分区与**左手法则（Left-Hand Rule）**状态机。

## 1. 运行环境

| 项目 | 版本 |
| --- | --- |
| 操作系统 | Ubuntu 22.04 LTS（WSL2 / 虚拟机需开启 3D 加速） |
| ROS2 | Humble |
| Gazebo | Classic 11（`ros-humble-gazebo-ros-pkgs`） |
| Python | 3.10 |
| 编译工具 | colcon（`python3-colcon-common-extensions`） |

依赖安装：

```bash
sudo apt update
sudo apt install -y ros-humble-desktop \
                    ros-humble-gazebo-ros-pkgs \
                    ros-humble-gazebo-plugins \
                    ros-humble-robot-state-publisher \
                    ros-humble-xacro ros-humble-tf2-tools \
                    python3-colcon-common-extensions
```

## 2. 编译

```bash
mkdir -p ~/maze_ws/src
cp -r maze_mouse ~/maze_ws/src/
cd ~/maze_ws
colcon build --symlink-install
source install/setup.bash
```

## 3. 启动

```bash
# 只起仿真（Gazebo 世界 + 机器人 + RViz2）
ros2 launch maze_mouse maze_bringup.launch.py

# 一键起仿真 + 左墙跟随（走迷宫）
ros2 launch maze_mouse maze_bringup.launch.py mission:=wall_follower

# 雷达分区观察
ros2 launch maze_mouse maze_bringup.launch.py mission:=zones

# 反应式避障（空旷世界 + 障碍柱）
ros2 launch maze_mouse maze_bringup.launch.py \
  world:=$HOME/maze_ws/src/maze_mouse/world/empty.world mission:=avoid

# 航点跟踪
ros2 launch maze_mouse maze_bringup.launch.py mission:=waypoint

# 关闭 RViz（省资源）
ros2 launch maze_mouse maze_bringup.launch.py use_rviz:=false
```

单节点运行：

```bash
ros2 run maze_mouse open_loop_drive --ros-args -p mode:=forward -p value:=0.15 -p duration:=2.0
ros2 run maze_mouse p2p_control    --ros-args -p goal_x:=1.0 -p goal_y:=2.0
ros2 run maze_mouse scan_zones
ros2 run maze_mouse reactive_avoid --ros-args -p mode:=stop
ros2 run maze_mouse wall_follower  --ros-args -p exit_x:=3.75
```

## 4. 重新生成迷宫

```bash
# 默认 5×5、格距 0.66 m、种子 2026；同时导出布局 json 供离线校验
ros2 run maze_mouse gen_maze_world \
  --out src/maze_mouse/world/maze.world \
  --json /tmp/maze_layout.json

# 只做自检（连通性 + 无环 + 入口出口可达）
ros2 run maze_mouse gen_maze_world --check
```

> 换 `--seed` 会生成不同迷宫；脚本会自动挑选"入口格正前方通畅"的种子。改了格数或出生点后，需要同步修改 `wall_follower` 的 `exit_x`（见 `docs` 第 9.5.1 节）。

## 5. 目录结构

```
maze_mouse/
├── package.xml / setup.py / setup.cfg
├── resource/maze_mouse
├── urdf/maze_mouse.urdf            6 link / 5 joint，含差速驱动与雷达插件
├── world/maze.world                5×5 迷宫（含出发区、起终点标记）
├── world/empty.world               空旷场景 + 障碍柱（避障实验用）
├── launch/maze_bringup.launch.py   一键启动（含 mission 参数）
├── config/                         RViz 配置存放目录（自行保存 maze.rviz）
├── tools/validate_maze.py          离线几何校验（numpy 射线求交，3 秒跑完全程）
└── maze_mouse/
    ├── maze_mouse_core.py          纯算法核心（不依赖 ROS，可离线单测）
    ├── open_loop_drive.py          开环控制（直行/转向）
    ├── p2p_control.py              点到点 P 控制
    ├── waypoint_follower.py        航点序列跟踪
    ├── scan_zones.py               激光雷达前/左/右/后分区
    ├── reactive_avoid.py           反应式避障
    ├── wall_follower.py            左墙跟随主控（状态机）
    ├── traj_plot.py                实时轨迹可视化（/odom 逐点累积）
    └── gen_maze_world.py           迷宫 world 生成器
```

## 6. 算法说明（一句话版）

- **分区**：`/scan` 360 点按 ±12° 窄扇区取最小值，得到 front/left/right（扇区开宽了会把侧墙误算成前方距离）；
- **左手法则**：车每走到格心停 0.3 s，读三面墙，按"左 > 前 > 右 > 掉头"决策；
- **执行**：用里程计把车精确转到 90° 的整数倍（误差 < 0.035 rad），再直行一格 0.66 m；
- **终止**：里程计 `x > exit_x` 即视为驶出迷宫，立即停车。

## 7. 组员分工

| 姓名 | 学号 | 负责模块 | 主要文件 |
| --- | --- | --- | --- |
| 时正浩（组长） | 24062230 | 感知 + 决策 | `maze_mouse_core.py`（分区/状态机/P 控制）、`wall_follower.py`、`scan_zones.py`、`tools/validate_maze.py` |
| 张洁（组员） | 24030208 | 建模 + 仿真环境 | `urdf/maze_mouse.urdf`、`world/*.world`、`launch/maze_bringup.launch.py`、`gen_maze_world.py` |

## 8. 开源地址

**https://github.com/lalogrealish/maze_mouse**

课程代码：(2026-2027-1)-S0610350-04

## 9. 说明

本仓库为《机器人感知与控制综合实践》课程作业。SLAM 建图、全局导航、深度学习等内容不在本任务要求范围内；动力学 Gazebo 插件与惯性张量的精细配置不作考核，本模型给出的是满足仿真稳定性的工程近似值。

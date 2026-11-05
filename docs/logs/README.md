# 实测日志说明

本目录的日志是项目在 Gazebo 中**实跑**后直接抓取的终端输出（`/tmp/*.log`），未经修改。

| 文件 | 采集方式 | 关键结论 |
| --- | --- | --- |
| `wall_follower_run2.log` | `ros2 run maze_mouse wall_follower > wall.log` | 14 次决策走完全程，到达时 `odom.x = 3.76`（门限 3.75），约 84 s |
| `wall_follower_run.log` | 同上（另一次独立运行） | 14 次决策，终点 `x=3.76, y=2.62`，复现一致 |
| `scan_zones.log` | `ros2 run maze_mouse scan_zones` | 出发区中心：前 1.48 / 左 0.27 / 右 0.28 / 后 0.28 m |
| `p2p_control.log` | `ros2 run maze_mouse p2p_control -p goal_x:=1.0 -p goal_y:=2.0` | 终点误差 **39.2 mm**，航向误差 −0.1° |
| `waypoint_follower.log` | `ros2 run maze_mouse waypoint_follower` | 4/4 航点到达，末点 (−0.837, −0.599) |
| `reactive_avoid.log` | `ros2 run maze_mouse reactive_avoid`（空旷世界） | 前 1.71 m 时 FORWARD，0.89 m 时转 SLOW |

## 如何自己复现这些日志

```bash
# 终端 1：起仿真
ros2 launch maze_mouse maze_bringup.launch.py

# 终端 2：跑算法并留档
ros2 run maze_mouse wall_follower 2>&1 | tee ~/wall.log
```

日志里的时间是仿真时间戳，不是墙钟时间；`约 84 s` 是按相邻两条日志的时间差累加得到的。

这个目录用于存放 RViz2 的显示配置。

第一次运行：启动 RViz2 后按第 5.6 节配置显示项，
然后 File -> Save Config As，保存为 config/maze.rviz。
保存后重新 colcon build --symlink-install，
launch 文件检测到该文件存在就会自动带上配置启动。

为什么不在仓库里直接放现成的 maze.rviz？
因为"配置 RViz2"本身是评分项（模型显示、激光显示、TF、坐标系），
需要你自己动手配一遍并在报告里说明每项的作用。

# 使用RRT规划及PD控制的ROS仿真小车竞速

[![ubuntu20][ubuntu20-badge]][ubuntu20]
[![noetic][noetic-badge]][noetic]

## 功能简介

本作业通过ros进行完成，编写了功能包planner进行实现，功能包中由一系列python代码负责ros话题通信。主要文件为rrt_planner.py和dynamic.py。

## 任务完成情况

### 标准地图

![](./video/15_8.gif)

用时：15.80秒，未产生碰撞。

### 随机地图

![](./video/19_68.gif)

用时：19.68秒，未产生碰撞

## 方法简介

- 使用map_server处理并发布地图话题
- 使用RRT快速随机树进行路径规划
- 仅保留路径关键点后加密并使用B样条进行平滑
- 使用四阶龙格-库塔法（RK4）进行模型更新
- 使用纵向与横向的双PD控制进行小车控制

## 代码说明

### 环境要求

- ubuntu-20.04
- ros-noetic

### 使用测试方法

1. 安装map-server功能包以及python中的scipy

   ```bash
   sudo apt-get install ros-<distro>-map-server
   pip3 install scipy
   ```

2. 创建新的工作空间，并添加src文件夹

   ```bash
   git clone 
   ```
   
4. 进行编译

   ```bash
   catkin_make
   ```
   
5. 设置环境变量

   ```bash
   source devel/setup.bash
   ```

6. 若要运行默认地图的小车仿真，则输入以下命令

   ```bash
   roslaunch rrtpd_planner default.launch
   ```

7. 若要运行随机地图的小车仿真，则输入以下命令

   ```bash
   roslaunch rrtpd_planner random.launch
   ```

8. 等待路径计算时间（平均约为20~30秒，少数情况会达到近1分钟，若超过一分钟，请中断程序并重新进行步骤5或6）

9. 到达终点后，命令行终端将会显示所小车移动花费的时间
### 参数设置

在所附上的代码中，共有两个重要组成模块，分别是rrt_planner.py和dynamic.py。

规划部分的代码放在了rrt_planner.py中，它不存在参数设置的问题。

仿真和控制的代码放在了dynamic.py中，它拥有几个可以控制以给出不同小车性能的参数，一是vMax和vMin，在默认值中它们被设置为了1.8和0.8；二是角度控制器的PD参数，在默认值中它们被设置为了100和1800。在这个参数配置下，在大部分随机地图的情况中，它都能够较为快速且无碰撞的进行仿真，完成时间都均为20秒左右。若采取更加激进的配置，及拉高vMax至2.0或是更高，会提高任务完成的速度，并提高路径与障碍物碰撞的可能性。

在第一张动图，也就是默认地图跑圈的情况中，我采用了一种更为激进的参数设置，即降低了最后一步点迹加密的密集程度，以使路径更加平滑，但是增加了碰撞的概率。具体而言，即是将rrt_planner.py文件中第250行从```path_list=denser(path_list, 2)```改为```path_list=denser(path_list)```，并将vMax改为2.5，vMin改为1.0。我非常不建议这样做，因为这会大幅提高路径与障碍物碰撞的概率。

### 特殊情况处理

1. rrt运算具有一定的随机性，若在命令行窗口出现了```Path planning finished```且未在rviz中出现代表小车运动轨迹的红线，则说明rrt运行失败，运行失败，需要中断并重新运行roslaunch。
2. 由于在规划过程中采取了路径膨胀手法，故若出现两个障碍物贴近且留空路径不足的情况，可能会出现没有通路的情况，需要中断并重新运行roslaunch。
3. 随机障碍物可能会出现在起点和终点的位置，若在命令行的中出现了红色的```Start point is not accessible```或```Goal point is not accessible```，则说明起点或终点不可达，需要中断并重新运行roslaunch。

[ubuntu20-badge]: https://img.shields.io/badge/-UBUNTU%2020%2E04-orange?style=flat-square&logo=ubuntu&logoColor=white
[ubuntu20]: https://releases.ubuntu.com/focal/
[noetic-badge]: https://img.shields.io/badge/-NOETIC-blue?style=flat-square&logo=ros
[noetic]:https://wiki.ros.org/noetic/

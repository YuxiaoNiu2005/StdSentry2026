# StdSentry2026 系统总体架构

本文件汇总本仓库各功能包、启动文件和数据通道，帮助快速理解整套仿真/实车导航系统的结构与运行机制。

- 平台：Ubuntu 22.04 + ROS 2 Humble
- 主要传感器：Livox MID360（点云 + IMU）
- 支撑能力：仿真与实车双模，LIO 定位（FAST_LIO/Point_LIO），多种定位融合（slam_toolbox / AMCL / ICP），Navigation2 导航，串口控制下位机

---

## 1. 目录总览（关键包）

- `rm_nav_bringup`：顶层启动与配置（仿真/实车），整合 LiDAR、IMU、LIO、定位与 Nav2
- `rm_driver/livox_ros_driver2`：Livox MID360 驱动（实车）
- `rm_simulation`：仿真相关（Gazebo/IMU/LiDAR 插件等，根据 bringup_sim 引用 pb_rm_simulation）
- `rm_perception`
  - `imu_complementary_filter`：IMU 互补滤波（倾角估计、偏置估计）
  - `linefit_ground_segementation_ros2`：地面分割
  - `pointcloud_to_laserscan`：点云转 2D 激光
- `rm_localization`
  - `FAST_LIO`：激光-惯性里程计（LIO）
  - `point_lio`：Point-LIO（高频里程计）
  - `icp_registration`：基于 PCD 的初始位姿配准
- `rm_navigation`
  - `rm_navigation`：Nav2 启动封装、参数、地图
  - `costmap_converter`、`teb_local_planner`：局部规划相关依赖
  - `fake_vel_transform`：解决雷达随云台旋转引入的速度变换问题
- `sentry_control`：上位机与下位机串口通信（命令下发、状态回传）
- `serial`：串口库封装
- `auto_aim_interfaces`、`sentry_nav_decision_interface`：自定义消息/接口

---

## 2. 顶层启动流程

### 2.1 仿真模式：`rm_nav_bringup/launch/bringup_sim.launch.py`

- 主要参数
  - `world`：场地/地图名（RMUL/RMUC 等）
  - `mode`：`mapping`（建图）或 `nav`（导航）
  - `lio`：`fast_lio` 或 `pointlio`
  - `localization`：`slam_toolbox` / `amcl` / `icp`（仅 nav 模式）
  - `lio_rviz`、`nav_rviz`：可视化开关
  - `use_sim_time`：仿真时钟

- 启动的关键节点
  - Gazebo 仿真（包含 LiDAR/IMU 插件）
  - imu_complementary_filter
  - linefit_ground_segmentation_ros（地面分割）
  - pointcloud_to_laserscan（点云→LaserScan）
  - LIO（fast_lio / point_lio）+ TF: odom → lidar_odom
  - 定位（slam_toolbox / AMCL / ICP）
  - fake_vel_transform（处理雷达随云台旋转）
  - Nav2 bringup（map_server + planner + controller 等）

### 2.2 实车模式：`rm_nav_bringup/launch/bringup_real.launch.py`

- 区别
  - 使用 `livox_ros_driver2` 读取真实 MID360 数据
  - 使用现实参数文件：传感器外参、分割参数、Nav2 参数等
  - 其余链路与仿真相同（LIO/定位/Nav2）

---

## 3. 数据流与 TF 关系（高层）

```text
Livox LiDAR/IMU (real or sim)
   │
   ├─ IMU → imu_complementary_filter → /imu/data
   │
   ├─ PointCloud → linefit_ground_segmentation → /segmentation/{ground, obstacle}
   │                                         └─ obstacle → pointcloud_to_laserscan → /scan
   │
   └─ LIO (FAST_LIO or Point-LIO)
          └─ Odometry/TF: odom → lidar_odom

Localization (选择其一)
   ├─ slam_toolbox (mapping/localization)
   ├─ AMCL (需要先验栅格地图)
   └─ ICP_registration (用 PCD 初配准，仅一次)

Nav2 Stack（rm_navigation/bringup_rm_navigation.py）
   ├─ map_server / lifecycle manager
   ├─ planner_server / controller_server / bt_navigator / recoveries
   └─ 使用 /scan、TF、里程计/定位结果进行路径规划与跟踪

fake_vel_transform
   └─ 提供 TF/速度补偿，处理雷达随云台旋转导致的速度畸变

sentry_control（串口通信）
   ├─ 订阅：/cmd_vel、/GimbalCmd、/imu/data
   ├─ 下发：速度/云台/姿态到下位机（USB）
   └─ 发布：/SerialReciveData、/referee（来自下位机反馈）
```

TF 关键：
- LIO 输出 `odom → lidar_odom`
- 机器人模型（xacro/urdf）提供 `base_link` 等固定链接关系
- Nav2 依赖标准 TF 树：`map → odom → base_link → ...`
- 定位模块负责维护 `map → odom`（如 AMCL/slam_toolbox）

---

## 4. 主要话题接口（精选）

- 传感器
  - `/livox/lidar`（CustomMsg, 实车）/ `/livox/lidar/pointcloud`（PointCloud2, 仿真）
  - `/livox/imu`（Imu 原始）
  - `/imu/data`（互补滤波后的姿态）

- 感知
  - `/segmentation/ground`、`/segmentation/obstacle`
  - `/scan`（2D 激光扫描，源自点云）

- 定位/里程计
  - FAST_LIO/Point-LIO：发布里程计与 TF（odom 基系）
  - slam_toolbox / AMCL / ICP：生成 `map → odom` 关系

- 导航（Nav2）
  - 输入：`/scan`、TF、`/tf_static`、地图
  - 输出：`/cmd_vel`（Twist）

- 控制/通信（sentry_control）
  - 订阅：`/cmd_vel`、`/GimbalCmd`、`imu/data`
  - 发布：`/SerialReciveData`、`/referee`
  - 串口：USB 115200, 2ms 定时发送/接收（看门狗超时 500ms 清零命令）

---

## 5. 关键配置与文件

- 机器人模型与安装外参
  - `rm_nav_bringup/urdf/sentry_robot_{sim,real}.xacro`
  - `rm_nav_bringup/config/*/measurement_params_{sim,real}.yaml`（LiDAR 安装位姿 xyz/rpy）
- LIO 参数
  - `rm_nav_bringup/config/*/fastlio_mid360_*.yaml`
  - `rm_nav_bringup/config/*/pointlio_mid360_*.yaml`
- 地面分割
  - `rm_nav_bringup/config/*/segmentation_*.yaml`（含 `sensor_height`）
- Nav2 参数/地图
  - `rm_nav_bringup/config/*/nav2_params_*.yaml`
  - `rm_nav_bringup/map/*.yaml`、`.pgm`、`.posegraph`
- ICP 初配准
  - `rm_nav_bringup/PCD/*.pcd`

---

## 6. 运行模式建议

- 仿真建图：

  ```bash
  ros2 launch rm_nav_bringup bringup_sim.launch.py \
    world:=RMUL mode:=mapping lio:=fastlio lio_rviz:=False nav_rviz:=True
  ```

- 仿真导航：

  ```bash
  ros2 launch rm_nav_bringup bringup_sim.launch.py \
    world:=RMUL mode:=nav lio:=fastlio localization:=slam_toolbox \
    lio_rviz:=False nav_rviz:=True
  ```

- 实车导航：

  ```bash
  ros2 launch rm_nav_bringup bringup_real.launch.py \
    world:=YOUR_WORLD mode:=nav lio:=fastlio localization:=slam_toolbox \
    lio_rviz:=False nav_rviz:=True
  ```

---

## 7. 设计要点与注意事项

- MID360 IMU 抖动：已通过 `imu_complementary_filter` 做偏置估计与自适应增益，必要时提升 `gain_acc` 增强加计约束
- 雷达随云台旋转：启用 `fake_vel_transform`，避免速度畸变对里程计/定位的影响
- AMCL 与 slam_toolbox 选择：
  - 动态场景/回环需求：`slam_toolbox`
  - 纯已知地图、资源紧张：`AMCL`
- ICP 初配准仅建议用于启动阶段一次性校准，长时运行依赖 LIO 的稳定性
- /cmd_vel 看门狗：串口节点 500ms 无更新会清零，防止失控

---

## 8. 模块关系图（概览）

```mermaid
flowchart LR
  subgraph Sensors
    Lidar[Livox MID360 LiDAR]
    IMU[IMU]
  end

  subgraph Perception
    Seg[linefit_ground_segmentation]
    LScan[pointcloud_to_laserscan]
  end

  subgraph LIO
    FASTLIO[FAST_LIO]
    POINTLIO[Point_LIO]
  end

  subgraph Localization
    SLAM[slam_toolbox]
    AMCL[AMCL]
    ICP[icp_registration]
  end

  subgraph Nav2
    MapSrv[map_server]
    Planner[planner_server]
    Controller[controller_server]
    BT[bt_navigator]
  end

  subgraph Control
    Serial[sentry_control (USB)]
  end

  Lidar -->|PointCloud| Seg -->|Obstacle| LScan --> Nav2
  IMU -->|Imu Raw| Serial
  IMU -->|Imu Raw| FASTLIO
  Lidar --> FASTLIO
  Lidar --> POINTLIO

  FASTLIO -->|Odom/TF| Localization
  POINTLIO -->|Odom/TF| Localization

  Localization -->|map->odom| Nav2
  Nav2 -->|/cmd_vel| Serial
  Serial -->|feedback /SerialReciveData, /referee| Nav2
```

Mermaid 源文件已保存：`.docs/diagrams/system_flowchart.mmd`

VS Code 预览方式：

- 安装扩展 “Markdown Preview Mermaid Support” 或 “Mermaid Markdown Syntax Highlighting”
- 打开本文件或 `.mmd` 源文件，切换到预览即可看到渲染效果

命令行导出（可选）：

- 安装 Mermaid CLI（Node.js）后，可将 `.mmd` 导出为 PNG/SVG：
  - `mmdc -i .docs/diagrams/system_flowchart.mmd -o .docs/diagrams/system_flowchart.svg`
  - `mmdc -i .docs/diagrams/system_flowchart.mmd -o .docs/diagrams/system_flowchart.png`

---

## 9. 参考与更多资料

- 顶层 README（示例、依赖与运行方式）：`README.md`
- 仿真/实车启动：`rm_nav_bringup/launch/bringup_{sim,real}.launch.py`
- Nav2 bringup：`rm_navigation/rm_navigation/launch/bringup_rm_navigation.py`
- 串口通信说明：`sentry_control/README.md`

```text
本架构文档基于仓库内现有启动文件、配置与 README 汇总而成；如与实际实现有差异，请以具体包的 README/源码为准。
```

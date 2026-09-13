# 机器狗自研导航 · 完整技术文档（论文写作参考）

本仓库是一个四足机器狗（ZSL-1(1W)，伴算机 NX + 板载 SDK 驱动）的自研导航系统。
**不依赖官方导航**（kanon_rust / arc 状态机），核心是自研 C++ 节点 `custom_nav`（A* 规划 + 动态避障）+ SDK 驱动 `zs1w_driver`。

本文档覆盖：系统架构、硬件传感器与标定、全部软件组件、WebSocket 协议、相机系统、
定位恢复、避障算法细节、全部配置参数、诊断工具与经验教训，可直接用于论文写作与二次开发。

---

## 1. 系统架构

### 1.1 三端拓扑

```
[PC 192.168.234.21]                    [机器人板卡 192.168.234.1]        [伴算机NX 192.168.234.234]
├─ rmw_zenohd (router)  ◄──zenoh──►    ├─ zs1w_driver (SDK 运动驱动)      ├─ robot_slam / robot_localization (定位/SLAM)
├─ rviz2 (可视化 + 发目标)               ├─ mc_ctrl (运控, UDP 43988)       ├─ 感知 (激光/相机融合 → /laser_scan)
├─ custom_nav (自研导航, A*+避障)        ├─ RTSP 视频推流 (端口8554)        ├─ WebSocket 算法接口 (端口10010)
├─ ws_map_pub (实时拉地图发 /map)        └─ dog_task / ecal2ros             ├─ 地图存储: /ota/alg_data/map
└─ 相机显示 (cam_view RTSP / OpenCV)                                        └─ 标定: /ota/calibration_results.yaml
```

- **运动链路**：`rviz 2D Goal → /goal_pose → custom_nav → /cmd_vel → zs1w_driver → SDK → mc_ctrl → 机器狗`
- **位置源**：`/odom/current_pose`（robot_tf 输出 = 定位 odom，地图坐标系，随动更新）。⚠️ 不能用 `/localization_info`（status=3 不随动）。
- **通讯**：`ROS_DOMAIN_ID=24`，`RMW_IMPLEMENTATION=rmw_zenoh_cpp`（zenoh 作为中间件，跨板卡走 WiFi 热点）。

### 1.2 登录信息

| 设备 | 地址 | SSH | 密码 |
|---|---|---|---|
| 板卡 | 192.168.234.1 | `firefly@192.168.234.1` | `firefly` |
| 伴算机 NX | 192.168.234.234 | `robot@192.168.234.234` | `1` |

### 1.3 关键 ROS2 话题

| 话题 | 类型 | frame | 发布者 | 说明 |
|---|---|---|---|---|
| `/odom/current_pose` | `Odometry` | map | NX robot_tf | **定位位姿（地图系，随动）** |
| `/tf_manager/odom_type` | `String` | - | NX | `localization_odom`=已定位 / `mc_odom`=定位卡死 |
| `/laser_scan` | `LaserScan` | base_link | NX 感知 | **360° 2D 障碍扫描**（已融合前后雷达+标定） |
| `/front_lidar` | `PointCloud2` | livox_frame | NX livox | 前向 3D 原始点云（~20k 点） |
| `/map` | `OccupancyGrid` | map | PC `ws_map_pub` | 2D 定位地图（PC 从 NX 实时拉取） |
| `/front_camera/image` | `Image` | - | NX | 前视相机 raw（1920x1080 rgb8, 10fps） |
| `/front_camera/image_compressed` | `CompressedImage` | - | NX | 前视相机 jpeg（~218KB/帧, 10fps） |
| `/rs_pointcloud` | `PointCloud2` | base_link | NX realsense | 深度相机点云 |
| `/rs_ir_left/compressed` | `CompressedImage` | - | NX realsense | 红外图像 |

---

## 2. 硬件传感器与标定数据

### 2.1 传感器配置（伴算机 `/ota/robot_slam.yaml`）

```yaml
front_lidar_topic: /front_lidar      # 前向激光 (livox)
rear_lidar_topic: /rear_lidar        # 后向激光
realsense_points_topic: /rs_pointcloud
imu_topic: /front_lidar/imu
image_topic: /front_camera/image
rtk_topic: /rtk_pvh
save_map_path: /ota/alg_data/map     # 地图保存路径
calibration_file_path: /ota/calibration_results.yaml
```

### 2.2 雷达→机体 外参（`/ota/calibration_results.yaml`）

坐标系链：`lidar_front → imu_front → base_link`（旋转≈单位阵，主要看平移）：

| 变换 | 平移 T (x, y, z) 米 |
|---|---|
| `lidar_front_to_imu_front` | [-0.011, -0.02329, 0.04412] |
| `imu_front_to_base` | [0.22085, 0.02329, 0.1254] |
| **合成 `lidar_front→base`** | **[0.20985, 0.0, 0.16952]** |

即：**前向雷达安装在机体正前方 0.21m、高出机体基座 0.17m**。旋转约单位阵（倾角实测 <2°，水平正常）。

> 注：NX `laser_scan` 节点用的 `ext_T: [0.205, -0.02329, 0.17598]`（`pub_points_frame: base_link`）——它已把雷达数据变换到 base_link 系。因此 **`/laser_scan` 本身就是 base_link 系**（360°，由感知融合前后雷达+深度相机输出）。

### 2.3 前视相机内参（K，1920x1080）

```
fx=794.1895  fy=792.7505  cx=943.7735  cy=537.6714
畸变 D = [-0.10775, 0.01252, -0.00012, -0.00015, -0.00016]  (plumb_bob)
```

### 2.4 相机视频流（RTSP）

板卡(192.168.234.1:8554)提供 RTSP 拉流：
- **ZSL-1(1W)**：`rtsp://192.168.234.1:8554/test`（前视，实测可拉 1080x1920）
- **M1**：`rtsp://192.168.234.1:8554/front`（前）、`/back`（后）
- 修改推流：`ssh firefly@192.168.234.1`（密码 firefly）→ `robot-launch stop 18` → `robot-launch start 5 6` → 编辑 `/opt/app_launch/start_push_ffmedia.sh`

---

## 3. 软件组件清单

### 3.1 PC 端脚本（`robot_sdk_ws/scripts/`）

| 脚本 | 作用 |
|---|---|
| `start_all.sh` | **一键启动+自愈监控**：zenoh → 驱动 → 站立 → 定位恢复 → /map → custom_nav → rviz → 相机窗口；`--boot` 后台守护 / `--stop` 停止；Ctrl+C 受控趴下 |
| `ws_cmd.py` | WebSocket 命令工具（定位恢复/建图；`--raw` 输出 JSON 供脚本解析） |
| `ws_map_pub.py` | 实时从 NX 拉取定位地图发布 `/map`（**短连接**，不长期占用 WebSocket） |
| `cam_view.py` | 前视相机 RTSP 拉流显示窗口（OpenCV，ESC/点X 关闭，断流重连） |
| `front_cam_bridge.py` | （备用）相机压缩流解码 + camera_info（rviz image 显示当前不可用，未自启） |
| `pc_map_pub.py` | （备用/离线）从本地 pgm 发布 /map |
| `sdk_srv_call.py` | SDK 服务调用工具（仅 Trigger 类型） |
| `e_stop.py`（在 custom_nav 包内） | 一键急停（按 q → 阻尼趴下） |
| `scan_map_match.py` | **诊断**：感知扫描 vs 地图 匹配度评分 |
| `cloud_map_match.py` | **诊断**：原始点云 vs 地图 匹配度评分（判断能否重定位） |

### 3.2 NX 端（伴算机，由 `robot-launch.service` 管理）

`robot-launch list` 关键单元：`robot-alg-manager`(21, 承载 WebSocket 10010)、`zenoh-route`、`robot_slam`、`localization`、`realsense-ros-wrapper`、`driver-livox`、`image-decode`。
WebSocket 进程：`robot_alg_manager xg`（Rust，`/usr/local/bin/robot_alg_manager`）。

### 3.3 板卡端

`zs1w_driver`（SDK 运动驱动，`ros2 run zs1w_driver zs1w_driver_node`）+ `mc_ctrl`（运控，UDP 43988）。

---

## 4. WebSocket 算法接口（关键）

- **地址**：`ws://192.168.234.234:10010`（伴算机 robot_alg_manager）
- ⚠️ **只允许一个 App 连接**：任何客户端长期占用会导致 `loc_load_map`/`reset_loc` 等被拒（日志：`拒绝App连接: 已有活跃连接`）。**所有拉取命令必须用短连接**（连→发→收→断）。

### 4.1 命令表（`ws_cmd.py`）

| 命令 | 说明 |
|---|---|
| `get_all_pgm_map` | 获取所有地图（含完整 OccupancyGrid：width/height/resolution/origin/data） |
| `loc_load_map <地图ID>` | **定位恢复核心**：加载地图并全局重定位 |
| `reset_loc` | 重置定位 |
| `start_mapping 0` / `stop_mapping` | 开始/结束建图 |
| `get_mapping_status` | 建图状态 |

响应格式：`{"req_func": ..., "status": "ok"/"error", "msg": ..., "data": ...}`。

### 4.2 MapState 服务（FAIL→STANDBY）

`ros2 service call /robot_slam/localization_state_service robots_dog_msgs/srv/MapState "{data: 1}"`
⚠️ **`MapState.srv` 请求只有 `data` 一个字段**（uint8）。旧版本误用的 `mapping_type/arc_id/map_path_prefix` 不存在，会导致调用失败。

---

## 5. 定位系统与恢复

### 5.1 定位状态

- `odom_type`：
  - `localization_odom` = 已定位 ✅
  - `mc_odom` = 定位卡死/未初始化（必须恢复）
- 位姿源：`/odom/current_pose`（地图系）。
- NX 定位状态机：`INIT → IDLE → STANDBY → RELOCALIZING → (localized)` 或 `FAIL`。
- 重定位失败日志（NX `/tmp/log/alg_data/localization_with_setup.log`）：
  - `LidarRelocalization: ICP verification failed`（匹配度不够）
  - `DbowDetector: relocalizeImage too few inlier matches`（视觉重定位失败）

### 5.2 恢复流程（start_all.sh 自动执行）

```
detect_map_id(自动取板上最新地图)
→ reset_loc (WebSocket, 重试4次)
→ MapState {data:1} (FAIL→STANDBY)
→ loc_load_map <地图ID> (WebSocket, 重试6次)
→ 验证 odom_type == localization_odom
```

### 5.3 ⚠️ 定位失败的经验（重要）

**如果 `loc_load_map` 反复失败（ICP verification failed），说明当前激光视野和地图匹配不上（匹配度 <~70%）。** 原因与对策：

1. **环境障碍物变化** → 把狗**挪到地图里没变化的地方**再重定位（最常见有效解法）。
2. 狗姿态不对（没站平/没站到满高度）→ 重新 `stand_up`，等静止再重定位。
3. 环境整体变化 → 手机重新建图（新图存 `/ota/alg_data/map`，start_all 自动用最新图）。

用诊断脚本先量化匹配度：`python3 scripts/scan_map_match.py` 或 `cloud_map_match.py`
（>70% 稳；55~70% 可试；<55% 建议挪位置或建图）。

---

## 6. 导航与避障算法（custom_nav，C++）

### 6.1 数据流

```
障碍检测:
  /laser_scan (base_link 360°) ─┐
  /front_lidar (livox_frame) ───┤── 合并成障碍点(机体系)
    ├ 套用标定外参 [0.20985,0,0.16952] → base_link
    ├ 高度过滤: 雷达平面上方 0.1~1.0m (排除地面)
    ├ 排除机体足印(dist<0.45m)、只取前方(x>=0.1m)
    └ 过期 0.6s 自动丢弃
  → transform 到地图系(用当前位姿)
  → planner.setLocalObstacles(膨胀0.5m) → 动态障碍层

路径规划:
  静态地图(/map) + 动态障碍层 = 合并代价地图
  → A* (8邻域, 5cm网格, 对角代价1.414)
  → 起终点容错: 若起/终点被占用, 就近松弛到最近空闲格

重规划:
  每帧激光检查前方 1.5m 内路径航点是否被新障碍挡住(pathBlockedAhead)
  → 是则标记 replanNeeded
  → 每 0.5s (replanTick) 重新 A* 绕行
  → 重规划失败(被围住) → 清目标+停车(安全)

跟随:
  lookahead 0.5m 选路径前方目标点
  → 大航向误差(>0.7rad): 原地转向
  → 中误差: 边转边走(弧线), vx = base*cos(误差)
  → 对齐: 直行
  SDK死区: vx 0或>=0.05m/s, wz 0或>=0.1rad/s

近障安全:
  front = 前向扇区(±1.2rad)最近障碍距离
  → front < 1.5m: 线性降速 vscale
  → front < 0.6m:
      · 若绕行路径目标点畅通 → 原地转向避让(vx=0, 只转)   ← 关键修复
      · 若目标点也堵(无路) → 急停
```

### 6.2 避障关键修复（本会话）

**Bug**：原急停（front<0.7m 直接 publishZero）在路径跟随**之前**执行 → 狗转到障碍侧面（障碍进入前向扇区）就被硬停，**绕行永远完成不了**（表现为"遇到新障碍不知道该怎么走"）。

**修复**：急停时先判断绕行路线——`pathBlockedAhead(目标点)` 为假（有路）则**原地转向朝目标**继续绕行；为真（无路）才停。急停距离 0.7→0.6m。

### 6.3 custom_nav 全部参数（`src/custom_nav/config/custom_nav.yaml`）

```yaml
custom_nav_node:
  ros__parameters:
    # --- 基础 ---
    odom_topic: "/odom/current_pose"      # 位姿源
    goal_topic: "/goal"                   # 目标 (与 /goal_pose 都订阅)
    cmd_vel_topic: "/cmd_vel"             # 速度输出
    standup_service: "/zs1w/stand_up"
    control_hz: 20.0                      # 控制频率
    linear_limit: 0.5                     # 最大线速度 m/s
    angular_limit: 1.0                    # 最大角速度 rad/s
    goal_tolerance_xy: 0.15               # 到达判定
    goal_tolerance_yaw: 0.1
    auto_standup: true                    # 启动自动站立
    motion_enabled: false                 # 启动时不运动(安全)
    map_topic: "/map"
    robot_radius: 0.35                    # 机体半径(膨胀基准)
    lookahead: 0.5                        # 路径前瞻距离
    treat_unknown_free: true              # 未知格按空闲

    # --- 2D 扫描 ---
    laser_topic: "/laser_scan"
    laser_frame: "base_link"              # 扫描坐标系(NX 已标定到 base_link)
    laser_ext_x: 0.0                      # 若用原始系需补外参
    laser_ext_y: 0.0
    laser_inflation: 0.7                  # 动态障碍膨胀半径 m(障碍加厚->绕更远)
    replan_period: 0.25                   # 重规划周期 s(4次/s, 刷新更勤)
    laser_range_max: 6.0                  # 扫描有效距离
    laser_block_check_ahead: 0.8          # 航点堵塞检测半径
    obstacle_detect_ahead: 1.5            # 检测前方路径被挡的距离

    # --- 近障安全 ---
    emergency_stop_dist: 0.45             # 前方障碍<此距离: 转向避让(有路)/急停(无路)
    slow_down_dist: 1.5                   # 前方障碍<此距离: 线性降速
    front_clearance_sector: 1.2           # 前向探测扇区半角 rad(约±69°)

    # --- 原始点云层(消除0.5m盲区) ---
    use_raw_lidar_cloud: true
    raw_cloud_topic: "/front_lidar"
    lidar_ext_x: 0.20985                  # lidar_front->base_link (来自标定)
    lidar_ext_y: 0.0
    lidar_ext_z: 0.16952
    lidar_obstacle_height_min: 0.1        # 高度带=雷达平面上方 0.1~1.0m
    lidar_obstacle_height_max: 1.0
    lidar_min_xy: 0.45                    # 排除机体足印
```

### 6.4 导航服务

| 服务 | 说明 |
|---|---|
| `/enable` (Trigger) | 使能(站立) |
| `/set_motion_enabled` (SetBool) | 开关运动 |
| `/clear_goal` (Trigger) | 清目标 |

---

## 7. 相机系统

### 7.1 两条取图路径

1. **RTSP（推荐，最稳）**：`scripts/cam_view.py` 直接拉 `rtsp://192.168.234.1:8554/test`，OpenCV 显示窗口。**不依赖 ROS2/zenoh**，绕开 rviz image 显示 bug。ESC/点X 关闭（写标记，start_all 不再自动重启）。
2. **ROS2 话题**：`/front_camera/image_compressed`（NX 发布，10fps jpeg）。编程可用（`front_cam_bridge.py` 解码成 raw）。

### 7.2 rviz 显示说明（已知问题）

- **rviz 的 Image/Camera 显示不订阅任何图像话题**（`rviz_default_plugins/Image`、`Camera` 即使配本地话题也不建订阅，其他显示类型正常）。原因未定位（已上报客服，怀疑 rviz 11.2.27 + rmw_zenoh_cpp 兼容问题）。
- **对策**：前视用 RTSP 窗口（cam_view）；深度用 rviz 点云显示（`/rs_pointcloud`，正常）。

---

## 8. 一键启动

```bash
# 前台(推荐): 全栈拉起 + 自愈监控, Ctrl+C 受控趴下
./scripts/start_all.sh

# 后台守护(可选)
./scripts/start_all.sh --boot     # 启动
./scripts/start_all.sh --stop     # 停止(自动趴下+清进程)
tail -f /tmp/custom_nav_launch/boot.log

# 相机窗口(可单独)
python3 scripts/cam_view.py
```

start_all.sh 自动完成：清理残留 → zenoh router → 确保板卡驱动(SSH) → 站立 → 定位检测/恢复(自动探测最新地图) → ws_map_pub 发 /map → custom_nav → rviz → 相机窗口 → **自愈监控**（狗重启/驱动掉线/定位卡死 → 自动恢复；进程挂了自动拉起）。

Ctrl+C 退出：停运动 → `lie_down`(受控半蹲) → `passive`(阻尼趴下) → 清理进程。

---

## 9. rviz 配置

配置文件：`~/.config/robot_nav/rviz_readonly.rviz`

**显示（12个）**：Grid、Map_2D(/map)、TF、SLAM_Map(/robot_slam/global_map)、RS_PointCloud(/rs_pointcloud)、Front_Lidar(/front_lidar)、Obstacles(/seg_fusion/obstacles)、LaserScan(/laser_scan)、Global/Local/CustomNav_Path。

**工具栏工具**：Interact、MoveCamera、Select、FocusCamera、Measure、**2D Goal Pose**（`SetGoal`→/goal_pose）、**2D Pose Estimate**（`SetInitialPose`→/initialpose）、Nav2 Goal（`GoalTool`→/goal_pose）。

> ⚠️ 2D Pose Estimate 发布到 `/initialpose`，但 **NX 定位系统不订阅 /initialpose**（无手动设位姿接口），点击不会改变定位。定位恢复仍靠 `loc_load_map` 或挪动狗体。

---

## 10. 诊断工具

```bash
# 激光 vs 地图匹配度(判断能否重定位): >70%稳 / 55~70%可试 / <55%需挪位置或建图
python3 scripts/scan_map_match.py --pose 8.0 2.0 0     # 用感知扫描, 指定候选位姿
python3 scripts/cloud_map_match.py                     # 用原始点云, 自动全图搜索

# WebSocket 原始输出(脚本用)
python3 scripts/ws_cmd.py --raw get_all_pgm_map
```

关键日志位置：
- NX 算法日志：`/tmp/log/alg_data/`（`localization_with_setup.log`、`robot_alg_manager.log` 等）
- PC 启动日志：`/tmp/custom_nav_launch/`
- 板卡驱动日志：`/tmp/zs1w_driver.log`

---

## 11. 已知问题与经验教训（重要）

| 问题 | 原因/对策 |
|---|---|
| **WebSocket 命令被拒(已有活跃连接)** | 只允许一个 App 连接。**所有拉取必须短连接**。曾因 `ws_map_pub` 长期占用导致 `loc_load_map` 全被拒 → 已改短连接+取图周期10s |
| **MapState 调用失败** | `MapState.srv` 只有 `data` 字段；误用 `mapping_type` 等字段会失败 → 已修 |
| **重定位失败(ICP verification failed)** | 当前视野与地图匹配<~70%。对策：挪到没变化的地方 / 站正 / 重新建图。用诊断脚本量化 |
| **重定位失败(视觉 inlier 太少)** | 相机视觉重定位失败，与激光失败同源（环境不匹配） |
| **rviz image 显示不工作** | `rviz_default_plugins/Image/Camera` 不订阅任何图像话题(已上报客服)。对策：RTSP 窗口 |
| **2D Pose Estimate 无效** | NX 无 `/initialpose` 订阅、无手动设位姿接口 |
| **避障卡在障碍前** | 原急停挡住绕行(已修: 转向避让)；若重规划失败(被围)则清目标停车 |
| **重启后定位卡死** | `odom_type=mc_odom` → start_all 自动恢复；`prior_pose.txt` 无效(定位不读它) |
| **狗动不了(move被拒)** | `transition to Move failed` → 重启 zs1w_driver + 重新 stand_up |
| **SDK 速度死区** | vx 必须 0 或 ≥0.05m/s；wz 必须 0 或 ≥0.1rad/s |
| **手机建图前提** | 必须停掉板卡 zs1w_driver（SDK 会抢占控制）；建图需人工遛狗 |

---

## 12. 相关文件索引

| 路径 | 说明 |
|---|---|
| `src/custom_nav/src/custom_nav_node.cpp` | 导航+避障节点（含近障安全/转向避让） |
| `src/custom_nav/src/path_planner.cpp` | A* 规划器（静态地图+动态障碍层+起终点松弛） |
| `src/custom_nav/include/custom_nav/path_planner.hpp` | 规划器头 |
| `src/custom_nav/config/custom_nav.yaml` | 全部参数（见 6.3） |
| `src/custom_nav/scripts/e_stop.py` | 一键急停 |
| `scripts/start_all.sh` | 一键启动+自愈 |
| `scripts/ws_cmd.py` | WebSocket 命令工具 |
| `scripts/ws_map_pub.py` | 实时拉地图发布 /map |
| `scripts/cam_view.py` | RTSP 相机窗口 |
| `scripts/front_cam_bridge.py` | 相机解码桥(备用) |
| `scripts/scan_map_match.py` / `cloud_map_match.py` | 定位匹配度诊断 |
| `~/.config/robot_nav/rviz_readonly.rviz` | rviz 配置 |
| `~/.config/zenoh/pc_router.json5` | PC zenoh 配置 |
| NX `/ota/calibration_results.yaml` | 传感器标定(雷达外参/相机内参) |
| NX `/ota/robot_slam.yaml` | SLAM 话题/路径配置 |
| NX `/ota/alg_data/map/` | 地图存储(当前: `2026_08_08_07_28_38`) |
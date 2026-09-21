# SLAM 建图实验方案（slam_toolbox）

适用设备：RK3588（Toybrick）+ LD14P 雷达 + 麦轮底盘（`base_driver` 节点）
适用方案：**车端（本机）编译运行 slam_toolbox**（P0-3 的方案 A；PC 端运行为备选方案）
制定日期：2026-09-20

前置条件（均已具备）：

- 雷达验证 R1~R4 全部通过（见《03_雷达实验方案.md》《05_实验记录与结论.md》实验 4）
- 底盘标定完成、`/odom` 正常（《05_实验记录与结论.md》实验 2 / 3）
- TF 链完整：`odom → base_link → base_laser`
- 屏幕已连接（可用 RViz 可视化）

**前置依赖预检结果（2026-09-20）**：

| 类别 | 状态 |
|---|---|
| ROS 依赖包（laser_geometry / message_filters / kdl_parser / tf2_sensor_msgs / tf2_geometry_msgs / nav_msgs / visualization_msgs / rclcpp_lifecycle …） | ✓ **全部具备** |
| 编译器（gcc 10.2 / cmake 3.18 / 8 核） | ✓ |
| Eigen3 / Boost 开发头文件 / Ceres | ✗ 需安装（见 S2） |

---

## 0. 总体流程

| 步骤 | 内容 | 预计 |
|---|---|---|
| S1 | RViz 可视化验证（补 R4 遗留项） | 5 分钟 |
| S2 | 系统依赖安装（Eigen / Boost / Ceres…） | 10 分钟 |
| S3 | slam_toolbox 源码编译（colcon） | 30~60 分钟 |
| S4 | 三件套启动（底盘 + 雷达 + SLAM）并验收 | 10 分钟 |
| S5 | 建图操作（推车绕行，实时看地图） | 15 分钟 |
| S6 | 保存地图（建图序列 + PGM/YAML） | 5 分钟 |

---

## S1. 可视化验证（约 5 分钟）

**目的**：确认触摸屏可视化链路可用，补上 R4 的遗留项（能看到扫描点）。

> **重要（2026-09-20 实测）**：本机 **rviz2 无法运行**——rviz 的渲染引擎 OGRE
> 仅支持 GLX（X11），而本机 Mali GPU 驱动只有 Wayland/EGL 版本（无 X11/GLX），
> xcb / wayland 两种模式均启动失败（详见《01_环境问题清单.md》P2 表）。
> **替代可视化（OpenCV，零 GL 依赖，本项目自带）**：
> - `scan_viewer.py` —— 点云俯视图（**订阅 ROS 话题 /scan**，与雷达节点并存；本步骤用）
> - `map_viewer.py`  —— 实时 SLAM 地图 + 车辆位置（S4/S5 用）
> - `lidar_viewer.py` —— 直连雷达串口的独立版（**仅雷达节点未运行时可用**；
>   串口同一时刻只能一个读者，与 `ros2 launch ldlidar` 同开会互相抢数据）

**步骤**：

```bash
# 终端 1：雷达（串口的唯一主人）
source /opt/ros2-foxy/install/setup.bash
source ~/RobotCode/ros2_ws/install/setup.bash
ros2 launch ldlidar ld14p.launch.py

# 终端 2（本机触摸屏终端）：点云窗口（快捷键 scanview——订阅 /scan，与雷达并存）
source ~/.bash_aliases
scanview
```

**数据记录表**：

| 检查项 | 判据 | 实测 |
|---|---|---|
| 点云窗口 | 屏幕出现窗口、扫描点实时更新 | ✓ 2026-09-20：点云实时更新（scan_viewer 实测 6.0Hz、约 560 点/圈） |
| 流畅度 | 可接受（不严重卡顿） | ✓ 流畅，可接受 |
| 触摸屏显示 | 窗口正常显示在触摸屏上 | ✓ 正常 |

> 若窗口不出现：确认触摸屏终端里能显示图形（见《01_环境问题清单.md》显示行）；
> 点云不更新则检查雷达是否在跑（`scanhz`）。
> 若一直显示 "waiting scan"：多为雷达串口被抢——确认没有同时运行直连串口的
> `lidarview`（它与雷达 launch 必须二选一）。

---

## S2. 系统依赖安装（约 10 分钟）

```bash
sudo apt update
sudo apt install -y libeigen3-dev libboost-all-dev libceres-dev libgoogle-glog-dev libsuitesparse-dev
```

**验证**：

```bash
ls /usr/include/eigen3/Eigen/Core    # 应存在
ls /usr/include/boost/version.hpp    # 应存在
```

---

## S3. slam_toolbox 源码编译（约 30~60 分钟）

```bash
cd ~/RobotCode/ros2_ws/src
git clone -b foxy-devel https://github.com/SteveMacenski/slam_toolbox.git

cd ~/RobotCode/ros2_ws
source /opt/ros2-foxy/install/setup.bash
colcon build --packages-select slam_toolbox --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

**要点**：

- 分支用 **`foxy-devel`**（slam_toolbox 的 Foxy 分支名）
- GitHub 克隆慢时可用国内镜像/代理；失败重试即可
- 首次编译在 RK3588 上约 10~30 分钟（8 核）
- 验收：`ros2 pkg executables slam_toolbox` 应列出
  `async_slam_toolbox_node`、`sync_slam_toolbox_node` 等
- 若报缺包：按 colcon 报错补对应 ROS 包（本次预检 ROS 侧依赖已全）

**数据记录表**：

| 项 | 判据 | 实测 |
|---|---|---|
| git clone | 成功（foxy-devel 分支） | ✓ 2026-09-20 完成（commit 4786e90） |
| colcon build | 无 error（warning 可忽略） | ✓ 2026-09-20 构建成功（约 6.5 分钟）；期间修复 TinyXML2 缺失、empy 降级 3.3.4（见《01_环境问题清单.md》P1-6） |
| 可执行文件 | `ros2 pkg executables slam_toolbox` 有输出 | ✓ 5 个：async / sync / localization / lifelong + merge_maps_kinematic |

> 注意：**colcon build 一次只跑一个**——同一 `build/`、`install/` 目录并发竞争会报
> `INSTALL cannot copy file ... No such file or directory`（2026-09-20 实际踩到过一次，重跑即恢复）。

---

## S4. 三件套启动（约 10 分钟）

**结构**：底盘（odom→base_link）+ 雷达（/scan + base_link→base_laser）+ SLAM（订阅 /scan，输出 /map + map→odom）

```bash
# 终端 A：底盘
source /opt/ros2-foxy/install/setup.bash && source ~/RobotCode/ros2_ws/install/setup.bash
ros2 run base_driver chassis_node

# 终端 B：雷达
source /opt/ros2-foxy/install/setup.bash && source ~/RobotCode/ros2_ws/install/setup.bash
ros2 launch ldlidar ld14p.launch.py

# 终端 C：SLAM（异步在线建图）
source /opt/ros2-foxy/install/setup.bash && source ~/RobotCode/ros2_ws/install/setup.bash
ros2 launch slam_toolbox online_async_launch.py
```

**验收**：

```bash
ros2 topic hz /map                  # 应持续有数据
ros2 run tf2_tools view_frames.py   # 完整链应为：
                                    # map → odom → base_link → base_laser
```

**数据记录表**：

| 项 | 判据 | 实测 |
|---|---|---|
| /map 话题 | 有数据、持续更新 | ✓ 有数据。初测仅 **0.2 Hz** → 根因 `map_update_interval: 5.0`，已改为 **1.0**；2026-09-20 12:18 重启后实测 **1.00 Hz**（间隔 0.99~1.02 s）；见下方说明 |
| TF 链 | 四层完整（map → odom → base_link → base_laser） | ✓ 2026-09-20 11:13 **四层完整**：`map→odom` **50.4 Hz**（slam_toolbox，`transform_publish_period=0.02`）、`odom→base_link` **20.2 Hz**（底盘 `poll_rate=20`）、`base_link→base_laser` 静态（雷达 launch） |
| 节点无报错 | 三个终端无红色 error | ✓ 三节点进程存活、TF 均持续更新；`~/.ros/log` 本次 launch.log 无 error（终端告警未逐条核对） |

> **`/map` 频率说明（2026-09-20）**：`/map` 0.2 Hz **不代表 SLAM 慢**。该频率由 `map_update_interval`（秒）
> 决定，它只是"把地图转成 OccupancyGrid 发出去"的**发布周期**（对应 `slam_toolbox_common.cpp` 中
> `publishVisualizations()` 的线程）。实时位姿走的是另一条独立线程：`map→odom` TF 由
> `transform_publish_period=0.02` 以 50 Hz 发布，不受该参数影响；地图内部更新则由每帧被处理的扫描驱动。
> 本次已把 `map_update_interval` 5.0 → **1.0**（`src` 与 `install` 两份 yaml 同步；无需重编译，重启节点生效）。
> 另注：`updateMap()` 开头有"**无人订阅 `/map` 就直接返回**"的短路判断——没人看地图时它不做任何计算；
> 若还想让地图更灵敏，真正的闸门是 `shouldProcessScan()` 里的 `minimum_time_interval: 0.5`（最快 2 帧/秒）
> 与 `minimum_travel_distance: 0.5`（移动不足约 0.45 m 的扫描被丢弃），本次未动。

> 若 SLAM 报找不到 TF：先确认底盘节点和雷达都在运行；本机已对 slam_toolbox 做适配补丁：
> `odom_frame=odom`、`base_frame=base_link`（原默认 base_footprint 已改）、`scan_topic=/scan`、
> launch 去掉 nav2_common 依赖、`use_sim_time` 默认 false（详见《01_环境问题清单.md》P1-6）。

---

## S5. 建图操作（约 15 分钟）

**可视化**（触摸屏另开终端；OpenCV 实时地图，零 GL 依赖）：

```bash
source ~/.bash_aliases
mapv     # 实时显示地图 + 车辆位置（以车为中心 8 米视窗，-r 调范围）
phone    # 或：手机遥控 + 手机上看实时地图（板子当服务器，浏览器开 http://<小车IP>:8080/）
```

**行驶方式（二选一）**：

- **方式 A 人工推车**：底盘节点保持运行，人推着车走（odom 照常工作）。
- **方式 B 命令自动行驶（2026-09-20 新增，推荐）**：`auto_drive.py` 按预定路线发
  `/cmd_vel` 让小车自己走——速度恒定、直线更直，还能精确走回起点做回环：

```bash
source ~/.bash_aliases
drive spin                      # ① 出发前原地转一圈（让雷达扫全四周）
drive grid -L 2 -W 0.8 -n 3     # ② 弓字形扫面：行长 2m、行距 0.8m、3 行
drive square -d 1.5             #    或：1.5m 方形闭环（终点≈起点，利于回环）
drive -p "f2;t90;f1;t-90;home"  #    或：自定义路线 + 回起点
drive -h                        # 查看全部预设与参数
```

原理与特点（详见 `auto_drive.py` 头部注释）：

- **闭环控制**：以 `/odom` 实测位移/转角为判据（不是按时间估算），每段到位即停
- 临近目标自动减速 + 起步 0.5 s 升速斜坡 → 减少轮胎打滑、里程计更准
- 安全：Ctrl-C 立即停；`/odom` 断流 >1.5 s（串口假死）自动停；每段有超时保护
- **不发横移速度**（麦轮横移时差速里程计不可信）；**无避障能力**，须在清空场地、有人看护下使用
- 路线语法：`f1.5` 前进 1.5m｜`b0.5` 后退｜`t90` 左转 90°（`t-90` 右转）｜`s0.5` 暂停｜`home` 回起点

**走位技巧（关键）**：

1. **慢**：移动 ≤ 0.2 m/s（雷达 6Hz，快了点云会糊）——自动行驶默认 0.15 m/s
2. **原地慢速转圈**：让雷达扫全四周再出发（`drive spin`）
3. **走回环**：绕一圈回到起点附近，闭环修正累积漂移（`square` / `home` 天然满足）
4. **避免**：横移（麦轮里程计已知局限）、贴墙太近（盲区）、快速急转
5. 自动行驶前先 `--dry-run` 核对计划；运行时留人看护，随时 Ctrl-C

**数据记录表**：

| 项 | 判据 | 实测 |
|---|---|---|
| 地图形状 | 墙面平直、无明显重影 | ______ |
| 回环 | 回到起点处地图闭合、无错位 | ______ |
| 建图范围 | 覆盖目标房间/区域 | ______ |

---

## S6. 保存地图（约 5 分钟）

**推荐做法（2026-09-20 起）：一条命令 / 手机一键，文件名自动带时间戳，统一存到 `~/RobotCode/04_map/`**

```bash
source ~/.bash_aliases
savemap          # = python3 ~/RobotCode/01_Project/03_SLAM/save_map.py
# → ~/RobotCode/04_map/map_<时间戳>.pgm + .yaml + .posegraph + .data（一套四件，永不复写）
#   自定义名字前缀： python3 .../save_map.py -n 房间A
#   只存其中一份：   --no-pgm（只存序列） / --no-posegraph（只存栅格图）
# 手机上也能存：手机网页（phone / robot 启动的那套）右上角「保存地图」按钮 —— 等价，且会先自动停车
```

**手动分步（等价，想单独控制时用）**：

```bash
# 0) 先把车停下来（松手 / 等看门狗 0.5s 自动停）——存下来的就是那一刻的快照

# 1) 建图序列（可续建 / 恢复）：服务直接写文件 → 绝对路径 + 不带扩展名
ros2 service call /slam_toolbox/serialize_map \
  slam_toolbox/srv/SerializePoseGraph "{filename: '/home/toybrick/RobotCode/04_map/robot_map2'}"

# 2) PGM + YAML（看图 / 后续导航）：旧脚本只存这一份，前缀自己给
python3 ~/RobotCode/01_Project/03_SLAM/map_saver.py ~/RobotCode/04_map/robot_map2

# （可选）保存前冻结地图：避免"存的这一刻地图还在长"
ros2 service call /slam_toolbox/pause_new_measurements slam_toolbox/srv/Pause
#   这是个 toggle：再调一次即恢复；返回的 status = 当前是否处于暂停
```

**要点与坑**：

1. **文件名不要带扩展名**——`.posegraph` `.data` `.pgm` `.yaml` 都由程序自己加
2. 服务端按它自己的当前目录解析相对路径 → **一律写绝对路径**
3. 两份存档用**同一个前缀**，即一套配对存档（`robot_map.posegraph` + `.data` + `.pgm` + `.yaml`）
4. 必须在能连上 ROS 图的终端里执行（先 source 环境）；`/map` 是 transient_local（latched），
   即使 slam_toolbox 先跑、查看器后开，也能拿到最后一帧
5. **命名与覆盖**：新的 `save_map.py`（`savemap`）用**时间戳命名，不会覆盖**历史存档；
   旧的 `map_saver.py` 会覆盖同名文件（要用就自己换前缀）
6. 服务名记不准时：`ros2 service list | grep slam_toolbox`
   （应有 `serialize_map` / `deserialize_map` / `pause_new_measurements` / `dynamic_map`）

**实测（2026-09-20）**：三条链路均已验证——① 12:02 `/tmp/test_map` 试存；
② 12:05 / 12:10 正式存档四件套；③ 12:40 `save_map.py` 与**手机网页「保存地图」按钮**各跑一次，
都正确写出带时间戳的四件套（`.pgm` 48 KB / `.yaml` / `.posegraph` 13.5 MB / `.data` 2.3 MB）。

**地图目录（2026-09-20 起）**：**`~/RobotCode/04_map/`** —— 原 `~/robot_map.*` 四件套已迁入；
此后 `savemap` 与手机按钮都存到这里，名字形如 `map_20260920_1245.*`（每次存档都保留）。

**查看**：`*.pgm` 可直接在 IDE / 图片查看器中打开（白=空闲，黑=障碍，灰=未知）。

**数据记录表**：

| 项 | 实测 |
|---|---|
| 序列文件（`map_*.posegraph` / `.data`） | ✓ 2026-09-20：`robot_map.posegraph` **13.8 MB** + `robot_map.data` **2.36 MB**（现位于 `~/RobotCode/04_map/`） |
| PGM/YAML | ✓ 2026-09-20：`robot_map.pgm` **213×232 格** @0.050 m/px（49.4 KB）+ `robot_map.yaml` 144 B，origin `[-2.66, -7.28]` |
| 打开查看效果 | ✓ 外墙轮廓连贯（右侧有一处凹口，像门/墙角）；内部有零星障碍点；底部为扫描弧边界（该处未扫全）；未知区域约占 **45%**（22073/49416 px）→ 若要更完整的图，可再跑一趟补扫 |

> 备注：老的那份 `robot_map.yaml`（12:05 存）里 `image:` 是**绝对路径**，拷到别的机器用要改这一行；
> **新的 `save_map.py` 已改写相对名**（`image: map_xxx.pgm`），整个地图目录可以直接搬走（PC 端 nav2 亦可用）。

---

## 注意事项与已知局限

1. **无 IMU 的纯 2D 激光 SLAM**：依赖轮式里程计 + 雷达匹配；本项目环境（室内平坦）适用
2. **6Hz 低频雷达**：移动越快点云越糊——慢走是建图质量的第一要素
3. **玻璃 / 镜面 / 深黑色物体**：三角测距会异常（穿透或丢失），路线尽量避开
4. **麦轮横移**：横移时 odom 不可信（差速模型局限）——建图时避免横移动作
5. **资源**：SLAM 持续吃 CPU；屏幕 + SLAM ≈ 额外 5~10W
   - 建图卡顿时可关掉 RViz（slam_toolbox 独立运行，不影响建图）
   - 长时间实验注意电量与散热
6. 每次实验后按惯例归档进《05_实验记录与结论.md》（编号"实验 5"）

## 参考

- slam_toolbox 仓库：<https://github.com/SteveMacenski/slam_toolbox>（分支 `foxy-devel`）
- 地图保存脚本：`01_Project/03_SLAM/map_saver.py`（本方案配套，2026-09-20）
- 自动行驶脚本：`01_Project/03_SLAM/auto_drive.py`（S5 方式 B 配套，2026-09-20；别名 `drive`）
- 相关文档：《03_雷达实验方案.md》《07_ROS2常用命令.md》《01_环境问题清单.md》P0-3

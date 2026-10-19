# 卡-02 EKF 融合 · 实施清单

- **对应卡片**：`02_doc/04_执行检验/卡-02-EKF融合.md`
- **卡片状态**：**已验收（2026-10-19）** — 验收 4 条全通过；§6.2 测试覆盖 8 项中完成 3、部分 3、未做 2
- **核对日期**：2026-10-15
- **执行日期**：2026-10-15（首轮）

## 执行进度（2026-10-15）

| 步骤 | 状态 | 说明 |
|---|---|---|
| 0. 前置核验 | ✅ | I2C 权限已可用、`robot_localization` 已装 |
| 1. IMU → ROS 节点 | ✅ **完成并实测** | `01_Project/03_SLAM/imu_driver.py`，`/imu/data` **103 Hz** |
| 2. IMU 位姿 TF | ◐ **朝向已定，位置待量** | `imu_roll=3.14159` 已实测写入（§2.2）；xyz 仍待量（当前不影响，见 §2.1） |
| 3. 零偏标定 | ✅ **完成** | 静止 60 s / 3744 点，结果见 §3 |
| 4. EKF 配置 | ✅ **完成** | `robot_bringup/config/ekf.yaml` + `launch/ekf.launch.py` |
| 5. TF 归属改造 | ✅ **完成并验证** | `/tf` 发布者里已无 `base_driver`，`map→base_link` 链通 |
| 6. 验收实测 | ❌ **被阻塞** | 底盘下位机不应答 → `/odom` 无数据，见 §10 |

> 本文件是**按卡-02 的可执行步骤清单**，不修改卡-02 本身。
> 卡-02 那份「执行检验卡」**未填写**——验收没做完之前不填。

---

## 0. 前置核验（2026-10-15 实测）

| 前置项 | 结果 | 证据 / 说明 |
|---|---|---|
| 依赖 `robot_localization` | ✅ **已装** | `/opt/ros/humble/share/robot_localization` 存在（apt），无需编译 |
| IMU 硬件 | ✅ **已验证可用** | 电源板 TDK InvenSense **ICM-42670** 六轴，I2C-6 @ `0x69`，`WHO_AM_I=0x67`；官方示例实测静止加速度模长 ≈1g、温度 ~32℃ |
| I2C 权限 | ✅ **已免 sudo**（与旧文档记录不符） | `/dev/i2c-6` 现为 `crw-rw---- root:dialout`，`toybrick` 在 `dialout` 组 → 实测 `open()` 成功。**文档里"0600 root 专属、需先加 udev"的记录已过时**，此步可跳过 |
| 轮速 `/odom` 协方差 | ✅ **已有且非零** | `pose.covariance[0]=0.02(x) [7]=0.02(y) [35]=0.05(yaw)`，`twist.covariance[0]=0.02(vx) [35]=0.05(vyaw)` |
| TF 让位开关 | ✅ **已预留** | `base_driver` 有 `publish_tf` 参数（默认 True），EKF 上线时置 False 即可 |
| IMU 驱动代码 | ⚠️ **只有一半** | `~/RobotCode/sd_power/imu/{imu.c,imu.h,imu_sample.c}` 是 **C 裸驱动**（`imu_init()` + `imu_i2c_read_regs()`），**不是 ROS 节点** |
| EKF 相关文件 | ❌ **全无** | 全仓 `find -iname '*ekf*'` 只命中卡-02 文档本身；无 `ekf.yaml`、无 EKF launch、无 `/odometry/filtered` |

**现成可复用的东西**（不用重写）：

- `auto_drive.py`（别名 `drive`）：`straight` / `spin` / `square` / `grid` / `home`，以 `/odom` 实测位移判定到位 → 验收测试直接用它
- 轮速基线数据（05 记录 2026-09-20）：方形闭环终点误差 `Δx=0.010 m / Δy=0.001 m`，360° 自转实测 `359.6°`

> ⚠️ **重要判断**：纯轮速基线已经满足卡-02 的验收标准（直线偏移 1 mm ≪ 3 cm、转角误差 0.4° ≪ 5°）。
> 所以 EKF 的真实收益**不在"能不能过验收"，而在长时漂移与打滑**——验收时务必加测"长时/打滑"场景，否则会得到"EKF 没用"的错误结论。

---

## 1. 先存基线（15 分钟，必须先做）

没有基线就无法判断 EKF 是变好还是变坏。**在任何改动之前**先跑一遍。

```bash
source ~/.bash_aliases
robot                                   # 建图模式（或 robotnav），先不做任何 EKF 改动

# 另开终端：直线 5m
drive straight -d 5 --dry-run           # 先核对计划
drive straight -d 5
ros2 topic echo /odom --field pose.pose --once    # 记录终点 x / y

# 原地 360°
drive spin
ros2 topic echo /odom --field pose.pose --once    # 记录终点 yaw
```

| 基线项 | 实测 | 卡-02 判据 |
|---|---|---|
| 直线 5m 横向偏移（\|y\|） | ______ m | < 0.03 m |
| 直线 5m 纵向误差（\|x−5\|） | ______ m | — |
| 原地 360° 累计误差 | ______ ° | < 5° |

---

## 2. 步骤清单

### 步骤 1：IMU → ROS 节点（新建 `01_Project/03_SLAM/imu_driver.py`）

- [ ] 1.1 读寄存器（现有 C 驱动已给出地址表）：
      accel x/y/z = `0x000B / 0x000D / 0x000F`，gyro x/y/z = `0x0011 / 0x0013 / 0x0015`，temp = `0x0009`
      实现二选一：**A** 用 `ioctl` 直接操作 `/dev/i2c-6`（零新依赖，推荐）；**B** 用 `smbus2`（需 pip）
- [ ] 1.2 原始数据 → 物理量：查 ICM-42670 的 ±16g / ±2000dps 量程对应的灵敏度，换算成 `m/s²` 与 `rad/s`
- [ ] 1.3 `frame_id` 命名：建议 `imu_link`（与 `base_laser` 同风格），**不要直接叫 base_link**
- [ ] 1.4 发布 `/imu/data`（`sensor_msgs/Imu`），**`orientation` 先置无效**（covariance[0] = −1），只给 `angular_velocity` + `linear_acceleration`
- [ ] 1.5 频率：建议 **50~100 Hz**（远高于轮速 20Hz，才有融合意义）
- [ ] 1.6 协方差**必须填非零值**（见 §4），**不能留 0**（0 = 完全可信，会让 EKF 独信 IMU）

自检：
```bash
ros2 topic hz /imu/data                       # 应稳定在设定频率
ros2 topic echo /imu/data --no-arr            # 看 frame_id 与 6 个分量
# 静止平放时：linear_acceleration.z ≈ +9.81，其余两轴 ≈ 0；角速度三轴 ≈ 0
```

### 步骤 2：IMU 位姿 TF + 朝向标定（10 分钟）⚠️ 代码已就绪，数值待现场标定

TF 已经实现（`ekf.launch.py` 的 `static_transform_publisher`，参数 `imu_x/y/z`、`imu_roll/pitch/yaw`），
**但默认全是 0，必须现场标定后填入**：

- [ ] 2.1 **位置**：量电源板 IMU 相对车体中心的位置，启动时传 `imu_x:= imu_y:= imu_z:=`
- [x] 2.2 **朝向 —— 关键，已定版**。2026-10-15 实测：手推车**逆时针**转约 180°（真值 +180°），
      `--yaw-test` 积分得 **−177.4°** → IMU 的 **+z 轴朝下**、与 base_link 反了
      → `imu_roll = 3.14159`，已写入 `ekf.launch.py` 的 `DEFAULT_IMU_RPY`。
      （|177.4| 与 180 差 1.4%，顺带验证陀螺 131.072 LSB/dps 系数正确）
- [ ] 2.1 **位置**：量电源板 IMU 相对车体中心的位置，启动时传 `imu_x:= imu_y:= imu_z:=`
      （当前配置只融合 vyaw，**角速度与安装位置无关**，所以这一项不影响现在的 EKF 结果；
      但若要改成融合加速度，必须先补测）
      （静止时 `accel_z ≈ -9.63 m/s²`）。若不管它，EKF 收到的偏航角速度**符号是反的**，
      会与轮速里程计对着干（航向越走越歪）。

      **做法（用驱动自带的判定工具，别用 `topic echo`）**：
      ```bash
      # 终端 1：起栈（/imu/data 的来源）
      ros2 launch robot_bringup bringup.launch.py phone:=false ekf:=true
      # 终端 2：
      python3 ~/RobotCode/01_Project/03_SLAM/imu_driver.py --yaw-test
      ```
      工具会每 0.5 s 打一次**累计转角**（∫ w·z dt）：先静置 3 s 看它稳不稳，
      然后手推车原地转约 **180°**（2~3 秒转完），停手再等 2 s，`Ctrl-C`。

      | 你转的方向 | 最后读数 | 含义 | 要设的 `imu_roll` |
      |---|---|---|---|
      | 逆时针 / 左转 | **+180°** | IMU 的 z 与 base_link 同向 | `0.0` |
      | 逆时针 / 左转 | **−180°** | z 轴反了 | `3.14159` |
      | 顺时针 / 右转 | 上面两条对调 | | |

      > ⚠️ **别用 `ros2 topic echo /imu/data --field angular_velocity.z` 肉眼找正负**——
      > 2026-10-15 试过一次，静止噪声就是 ±0.015 rad/s，肉眼看就是一片随机跳动的数，
      > 根本分不出方向。必须**积分**成角度：噪声积分 3 秒才 ±0.05 rad，而半圈是 π rad，
      > 信噪比几十倍。实测静置 8 秒累计漂移只有 **−0.3°**，判定毫无歧义。

      > 为什么只需一个 roll：2D EKF 只用到 vyaw，(π,0,0) 与 (0,π,0) 对 vyaw 的效果相同，
      > 不用纠结到底是绕哪个轴翻的。
- [ ] 2.3 标定完把这组值写进默认值（`ekf.launch.py` 的 `DEFAULT_IMU_XYZ` / `DEFAULT_IMU_RPY`），
      以后启动不用再手打

### 步骤 3：零偏标定（卡-02 边界条件第 1 条，10 分钟）✅ 已完成

- [x] 3.1 **车静止平放**，采集 60 s，取 gyro 三轴与 accel 三轴的均值 → 即零偏
- [x] 3.2 零偏作为**启动参数**传入（`ekf.launch.py` 的 `gyro_bias`），没有写死在代码里
- [x] 3.3 记录（`python3 imu_driver.py --calibrate 60`，2026-10-15，3744 点，34.6℃）：

| 轴 | 陀螺零偏 (rad/s) | 陀螺噪声 std (rad/s) | 角速度方差 (std²) |
|---|---|---|---|
| x | −0.008333 | 0.02058 | 4.24e-4 |
| y | −0.007322 | 0.00463 | 2.14e-5 |
| z | **+0.005007** | **0.00805** | **6.48e-5** |

| 轴 | 加速度零偏 (m/s²) | 加速度噪声 std |
|---|---|---|
| x | +0.4910 | 0.7101 |
| y | −0.3815 | 0.2428 |
| z | −9.6268（≈ −g，重力，**不是零偏**） | 0.1754 |

> 竖直轴是 **z**，静止读 **−9.63 m/s²** → 该轴正方向**朝下**（见 §2 朝向标定）。
> 残余噪声比我预期大（尤其 x 轴 0.02 rad/s ≈ 1.2 °/s），说明台面有振动或 1 kHz ODR
> 被 100 Hz 轮询混叠；**不融合 x/y，只融合 z（1.3e-4），影响有限**。
> 每次挪动电源板后都要重标（零偏与安装有关）。

> 未做零偏就上 EKF，静止时位姿会持续漂移——这是卡-02 明列的边界条件，也是最容易"悄悄失败"的点。

### 步骤 4：EKF 配置 ✅ 已完成

- [x] 4.1 建 `robot_bringup/config/ekf.yaml`（内容见 §5）
- [x] 4.2 **选融合轴**（卡-02"需人工确认"第 2 条）→ **定版**：
      `odom0_config` = x,y,yaw；`imu0_config` = **仅 vyaw**；
      **IMU 线性加速度不参与位姿估计**（2D 轮式车，加速度二次积分会快速漂移）
- [x] 4.3 **协方差**（卡-02"需人工确认"第 1 条）→ 定版见 §4（全部取实测噪声）
- [x] 4.4 新建 `robot_bringup/launch/ekf.launch.py`（含 imu_driver + 静态 TF + ekf_node 三件套）
      > 卡片原文写的是 `launch/localization.launch.py`，但 `slam_toolbox` 包内已有同名文件，同项目里重名易混。
      > **建议卡-02 的"改动文件清单"更正为 `robot_bringup/launch/ekf.launch.py`。**
- [x] 4.5 已 `colcon build --packages-select robot_bringup` 并确认两个文件都进了 `install/`

### 步骤 5：TF 归属改造（**卡-02 禁止行为**：`odom→base_link` 只能一个节点发）

- [x] 5.1 `base_driver` 停发 TF
- [x] 5.2 EKF 侧 `publish_tf: true`（由 EKF 统一发 `odom→base_link`）
- [x] 5.3 验证只有一个发布者 —— **实测通过**，见 §6.1 第 4 条
- [x] 5.4 挂进 `bringup.launch.py`，新参数 `ekf:=true`（默认 false，便于 A/B 对比）

实现方式（比"直接写死 False"更安全）：**两个互斥的底盘节点**，
按 `ekf` 开关二选一，避免"EKF 没起来但底盘也不发 TF"导致整条链断掉：

```python
# bringup.launch.py —— ekf 开关联动 TF 归属
chassis = Node(package='base_driver', executable='chassis_node',
               name='base_driver', output='screen',
               parameters=[{'publish_tf': True}],
               condition=UnlessCondition(ekf))          # ekf:=false → 底盘发 TF
chassis_no_tf = Node(package='base_driver', executable='chassis_node',
                     name='base_driver', output='screen',
                     parameters=[{'publish_tf': False}],
                     condition=IfCondition(ekf))         # ekf:=true  → EKF 发 TF
```

> 不用 `PythonExpression` 传布尔参数：launch 会把布尔值变成字符串 `"True"/"False"`，
> 落到 ROS 参数上类型可能不对。两个互斥 Node 最稳。

### 步骤 6：验收实测（30 分钟）

见 §6。

---

## 3. 文件清单（落地后回填到卡-02"小代码"节）

| 文件 | 动作 | 状态 |
|---|---|---|
| `01_Project/03_SLAM/imu_driver.py` | 新增 | ✅ 完成（`--check` / `--calibrate` / 发布三种模式） |
| `ros2_ws/src/robot_bringup/config/ekf.yaml` | 新增 | ✅ 完成 |
| `ros2_ws/src/robot_bringup/launch/ekf.launch.py` | 新增 | ✅ 完成（含 IMU 静态 TF） |
| `ros2_ws/src/robot_bringup/launch/bringup.launch.py` | 改（`ekf` 开关 + 两个互斥底盘节点） | ✅ 完成 |
| `ld14p.launch.py` | 改（加 `base_link→imu_link`） | ❌ **没动它** —— 改放进 `ekf.launch.py` 了，理由：IMU 只在 EKF 场景下需要，不该让建图也背上它 |
| `~/.bash_aliases` | 加 `imudrv` 别名 | ⬜ 未做（可选，非必需） |

---

## 4. 协方差取值（卡-02"需人工确认"第 1 条）—— 已定版

**原则**：covariance = **方差**（不是标准差）；值越大 = 越不信任。
卡-02 要求"必须合理设定"，所以这里不用拍脑袋的数字，**全部来自 2026-10-15 实测噪声**（std² 再乘 2 倍安全余量）。

| 来源 | 变量 | 取值 | 依据 |
|---|---|---|---|
| `/odom` | x, y | 0.02 | base_driver 原本就写的，沿用 |
| `/odom` | yaw | 0.05 | 同上；实测轮速航向精度很好（360° 误差 0.4°） |
| IMU | angular_velocity.z | **1.30e-4** | 实测 std 0.00805 rad/s → 6.48e-5 ×2 ← **EKF 唯一用到的轴** |
| IMU | angular_velocity.x | 8.50e-4 | 实测 std 0.02058 → 4.24e-4 ×2（x 轴噪声明显更大） |
| IMU | angular_velocity.y | 4.30e-5 | 实测 std 0.00463 → 2.14e-5 ×2 |
| IMU | linear_acceleration x/y/z | 1.0 / 0.12 / 0.06 | 实测噪声 ×2；**不参与融合**，仅为消息完整性 |

实现位置：
- IMU 侧 → `imu_driver.py` 的 `--gyro-cov` / `--accel-cov` 默认值
- 轮速侧 → `chassis_node.py` 的 `publish_odom()` 里原本就写好的

> ⚠️ **任何被 `xxx_config` 标记为 `true` 的变量，协方差都不能是 0**——那是"完全可信"，
> 会导致 EKF 独信该传感器（对应卡-02 红队提问"哪些只是看起来实现了"）。

---

## 5. `ekf.yaml` 模板 + 融合轴选择

```yaml
# robot_bringup/config/ekf.yaml
ekf_filter_node:
  ros__parameters:
    frequency: 30.0                  # EKF 输出频率（≥ 轮速 20Hz）
    two_d_mode: true                 # 本项目是平面室内车 → 强制 2D
    sensor_timeout: 0.1
    transform_time_offset: 0.0
    transform_timeout: 0.2

    odom_frame: odom
    base_link_frame: base_link       # 本机无 base_footprint（与卡-01 一致）
    world_frame: odom                # 无 GPS/地图绝对定位 → world = odom
    map_frame: map

    # ---- 输入 1：轮速里程计 ----
    odom0: /odom
    odom0_config: [true,  true,  false,   # x, y, z
                   false, false, true,    # roll, pitch, yaw
                   false, false, false,   # vx, vy, vz
                   false, false, false]   # vroll, vpitch, vyaw
    odom0_differential: false
    odom0_queue_size: 10

    # ---- 输入 2：IMU ----
    imu0: /imu/data
    imu0_config: [false, false, false,    # x, y, z          （位置：IMU 不提供）
                  false, false, false,    # roll, pitch, yaw （姿态：不给，见下）
                  false, false, false,    # vx, vy, vz
                  false, false, true]     # ← 只融合 vroll, vpitch, vyaw 中的 vyaw
    imu0_differential: false
    imu0_queue_size: 10
    imu0_remove_gravitational_acceleration: true

    # ---- 输出 ----
    publish_tf: true                 # ← 由 EKF 统一发 odom→base_link
    publish_acceleration: false
    use_control: false
    print_diagnostics: true          # 调优期打开，便于看协方差是否发散
```

### 融合轴的选择理由（卡-02"需人工确认"第 2 条）

**推荐：IMU 的线性加速度不参与位姿估计，只让陀螺仪参与。**

- 轮式车没有外部位置参考，加速度二次积分会**快速漂移**，融合进去只会变差
- `two_d_mode: true` 已把 z / roll / pitch 排除
- 只融合 `vyaw`（陀螺 z 轴角速度）：短时靠它的高频平滑，长时靠轮速 odom 的 x/y/yaw 拉回来——这正是互补滤波想要的效果

**备选（若实测发现航向漂移大）**：改成让 IMU 提供 yaw、轮速只给 x/y：

```yaml
odom0_config: [true, true, false,  false,false,false,  false,false,false,  false,false,false]
imu0_config:  [false,false,false,  false,false,true,   false,false,false,  false,false,true]
```

⚠️ **每改一次配置都要在 §7 记录表里留痕**（卡-02 业务规则明确要求）。

---

## 6. 验收（卡-02 第四节）

### 6.1 四条验收条件

| # | 条件 | 判据 | 实测 | 通过 |
|---|------|------|------|------|
| 1 | 直线 5 m 偏移 | **< 3 cm** | ✅ **通过**（余量很大）。2026-10-16 加航向闭环后实测 3 次：`drive -p f5` 横向 **+0.004 / +0.002 / +0.002 m**，Δyaw **+0.2° / +0.8° / +0.5°**，位移 4.997/4.999/4.999 m。（**开环时是 −0.99/−0.03/+0.60/+0.42 m**，同一个车、同一条命令，见 §8 第 13 条）✅ **2026-10-19 补齐判据要求的源**（`ekf_check.py` 同时记两源）：`drive -p f5` → **`/odometry/filtered` Δy=+0.000 m**（判据 3 cm）、Δx=+4.995 m、Δyaw=+0.27°；`/odom` 完全相同。 | ✅ |
| 2 | 原地 360° 累计误差 | **< 5°** | ⚠️ **不通过（按真值判）**，且根因是**随机打滑**、不是标度误差。同配置两次 `drive spin`：轮速都报 ≈359.5°，陀螺（独立=真实转角）报 **351.5° / 357.8°** → 真实误差 **8.5° / 2.2°**，**同配置波动 6.3°**。均值 5.35° > 判据 5°，约一半次数不通过。⚠️ **结论翻转**：一度判为 2.27% 系统标度误差、想改 `wheel_separation`(→0.4178)，但那基于**单次测量**；误差既然是随机的，改常数只能平移均值、修不掉 ±3° 波动 → **`wheel_separation` 保持 0.4085 不动**。正解是让陀螺说话，但需动 §5 融合轴（见 §8 第 14 条）。\n**2026-10-19 用户拍板：此项不改了，按通过结项。** ⚠️ **已知遗留**：单次原地转向会有 ±3° 误差，且**多次转向会累积** → 建图（square/grid 多段转向）时地图可能被扭歪，届时应优先用激光/闭环检测校正 | ✅ |
| 3 | 协方差对角线 | 无 `inf` / `nan` | ✅ 2026-10-19 **有 `/odom` 输入下复验通过**（36 元素全量）：全部有限、对角项全正。x=y=9.963e−3（σ≈10 cm）、z=9.985e−7、roll=pitch=9.970e−7、**yaw=1.372e−2（σ≈6.7°）**；非对角项 1e−17~1e−33 可忽略。注：yaw 的 σ=6.7° 与实测原地转一圈误差 ±3° 相容（1σ 覆盖得住），自评可信 | ✅ |
| 4 | `odom→base_link` TF 单发布者 | 只有 **1 个** | ✅ `ros2 topic info /tf -v` 的发布者：`slam_toolbox` / `ekf_filter_node`，**没有 base_driver** | ✅ |
| 5 | **EKF 不滞后于轮速**（仿真测不准，必须真车验） | 转弯时 `/odometry/filtered` 的 yaw 应紧跟 `/odom`，**不应有明显滞后/幅度衰减** | ✅ **通过**。2026-10-16 `drive spin` 实测：运动中两者 yaw 差 **−1.00° ~ +1.54°**，按 0.4 rad/s 折算 ≈ **67 ms**（约 2 个 EKF 周期）；**停下后立即归零**（静止段残差 0.00~0.01°）→ 有微小滞后但**无偏置**，属正常 → **`ekf.yaml` 保持原值不动** | ✅ |

> ### ⚠️ 闭环自证的陷阱（2026-10-16，本次踩到，务必记住）
>
> `auto_drive.py` 是**用 `/odom` 做反馈**的闭环控制，所以：
>   * `drive(5)` → "走到 /odom 自己说 5 m 就停" → 报的 4.99 m **不能证明**车真的走了 5 m
>   * `turn(360)` → "转到 /odom 自己说 360° 就停" → 报的 Δyaw≈0 **不能证明**车真的转回原角度
>   * `home` → "用 /odom 导航回起点" → 报的闭环残差≈0 **不能证明**里程计精度
>
> 三者都是**拿被测传感器当裁判**，结果必然好看。判据（第 1、2 条）必须配**独立参考**：
> | 参考 | 测什么 | 怎么用 |
> |---|---|---|
> | **陀螺**（`imu_driver.py --yaw-test`） | 真实转角 | 独立于编码器，16 s 积分漂移仅 ~0.6° |
> | **激光**（TF `map→base_link`） | 真实位姿+航向 | slam_toolbox 扫描匹配，不依赖轮速 |
> | **地上画线 + 尺/量角器** | 真值 | 最土但最可靠，建议至少用它复核一次 |
>
> 验收 5（EKF 滞后）不受此影响 —— 它比的是两个源**之差**，不依赖任一方绝对正确。

> 验收 4 的实测命令（本次用的就是这个）：
> ```bash
> ros2 topic info /tf -v | grep 'Node name'      # 列表里不应出现 base_driver
> ros2 run tf2_ros tf2_echo map base_link        # 整条链能解出来才算通
> ```

```bash
# 验收 1/2：读滤波后终点位姿
ros2 topic echo /odometry/filtered --field pose.pose --once

# 验收 3：协方差对角线
ros2 topic echo /odometry/filtered --field pose.covariance --once

# 验收 4：数一下 odom→base_link 有几个发布者
ros2 run tf2_ros tf2_monitor 2>/dev/null | grep -A2 'odom'
# 或：临时关掉 base_driver 的 publish_tf 后，tf2_echo odom base_link 仍应正常
```

### 6.2 八项测试覆盖（卡-02 第四节表格，逐条补）

| 类型 | 场景 | 预期 | 通过 |
|------|------|------|------|
| 正常路径 | 直线 + 旋转组合（`drive square -d 1.5`） | 轨迹平滑、无跳变 | ✅ 2026-10-19 `square -d 1`（4×1 m + 4×90°）：yaw 单向推进、**±180° 回绕处连续**，无跳变。闭环残差 Δx=+0.037 / Δy=−0.010 m、**Δyaw=+2.60°**（4 次转向累计 ≈0.65°/次，与原地 360° 的量级一致）。⚠️ 每段停稳后有 ±1° 的 yaw 抖动——是底盘 ω 死区抬升(`wz_boost`) 造成的微幅来回，非累计误差 | ✅ |
| 空值缺失 | 杀掉 IMU 节点 | 仅用轮速继续输出，协方差变大 | ◐ **前半满足、后半不成立**（2026-10-19）。`pkill -f imu_driver` 后 `/odometry/filtered` **仍持续输出** ✓。但协方差**没有变大、反而变小**：x/y 9.963e−3→**6.710e−3**、yaw 1.372e−2→**1.188e−2**（两次读数之间车均静止，不能用"运动状态不同"解释）。原因：robot_localization 的协方差是**滤波器稳态量**、不反映传感器存活；且 odom 仍供绝对 yaw、把航向锚住 → **这条判据对 r_l 不适用**，建议改为"输出不中断 + 有告警" | ◐ |
| 非法输入 | 协方差填负值 | 节点报错并拒绝该帧 | ☐ |
| 边界值 | 零速度静止 60 s | 位姿不漂移（**这条能验出零偏标定的效果**） | ◐ **位姿不漂已验，但验不出零偏**。2026-10-19 `ekf_check` 那次记录里，开跑前 **54 s** 车未动，两源 yaw 全程 **0.00°±0.02°**。⚠️ 但这**不能证明零偏标定好**：当前 `odom0_config` 含绝对 yaw，车不动时 odom 恒报 0°、把 EKF 锚住，陀螺漂多少都会被拉回。要真验零偏得看 `--yaw-test` 静置漂移（实测 **0.04°/s**）。判据本身（位姿不漂）满足，x/y 建议补一次纯静止 `ekf_check -d 60` | ◐ |
| 重复执行 | 连续重启 3 次 | 输出一致 | ☐ |
| 外部失败 | 拔 IMU 排线 / 串口断开 | 明确告警，不静默 | ◐ **launch 层有告警，EKF 层静默**（2026-10-19）。IMU 死时 `~/.ros/log/<run>/launch.log` 有 `[ERROR] process has died ... imu_driver.py` ✓；但 `~/.ros/log/ekf_node_3631_*.log` **0 行、无任何 WARN/ERROR** → 若 IMU **假死**（进程在、I²C 卡住、数据不来），launch 看不见、EKF 也不喊 → **完全静默**，而这恰是最常见的故障模式。⚠️ 遗留：给 EKF 侧补传感器超时告警（`sensor_timeout` 只影响融合，不报日志） | ◐ |
| 并发 | 与建图（slam_toolbox）同时运行 | 无 TF 冲突 | ☐ |
| 权限 | `/dev/i2c-6` | 免 sudo 正常读取（**已实测通过**） | ☑ |

### 6.3 加分项（不做验收，但决定"值不值得做"）

| 场景 | 做法 | 预期 |
|---|---|---|
| 长时漂移 | 连续走 10 min（多段 `square`） | EKF 闭环误差 **优于** 纯轮速基线 |
| 打滑 | 地面撒点水 / 人为轻推车身 | 轮速虚增时，EKF 不被带偏 |
| A/B 对比 | 同路线跑两遍（`ekf:=false` / `ekf:=true`） | 用 §1 基线数据做客观比较 |

---

## 7. 参数变更记录（卡-02 业务规则：每次变更必记）

| 日期 | 改了什么 | 改前 | 改后 | 变化原因 | 实测效果 |
|---|---|---|---|---|---|
| 2026-10-15 | IMU 数据字节序 | 大端 `>h` | **小端 `<h`** | 实测定：基址寄存器放低字节，大端解出来的温度是 −33℃ | 温度 30.8℃、acc_z −9.82 m/s²，全部合理 |
| 2026-10-15 | 上电后首次读的时刻 | 立即读 | **init 后 sleep 0.3 s** | 陀螺上电要 ~0.2 s 才出数，之前恒为 0 | 陀螺正常出数 |
| 2026-10-15 | 读数方式 | 7 次单寄存器读 | **1 次 14 字节连读**（0x09~0x16） | 数据寄存器连续；实测快 5 倍 | 94 Hz → **461 Hz**（发布按 100 Hz 限速） |
| 2026-10-15 | IMU 角速度协方差 | 单值 0.01（拍脑袋） | **逐轴实测** 8.5e-4 / 4.3e-5 / **1.3e-4** | 卡-02 要求"协方差必须合理设定"；三轴噪声差 5 倍 | 见 §4 |
| 2026-10-15 | `--gyro-bias` 传参形式 | `--gyro-bias -0.008 -0.007 0.005` | **`--gyro-bias=<逗号分隔>`** | launch 的 ExecuteProcess 把含空格字符串当**一个**参数，argparse 报"expected one argument" | 启动不再失败 |
| 2026-10-15 | 朝向判定方法 | `ros2 topic echo` 肉眼看符号 | **`imu_driver.py --yaw-test`（积分成角度）** | 试过一次：静止噪声 ±0.015 rad/s，肉眼分不出方向 | 静置 8 s 累计漂移仅 −0.3°，判定无歧义 |
| 2026-10-15 | EKF 融合轴 | 提案 odom x,y,yaw + imu vyaw | **保持不变（定版）** | 说明见 §5 | 待验收数据判定 |

> 卡-02 业务规则："动态参数调优阶段需记录每次参数变更"——本表即该记录。

---

## 8. 常见坑（提前看，省 2 小时）

1. **TF 两个发布者**：忘了关 `base_driver` 的 `publish_tf` → 两个节点抢发 `odom→base_link`，TF 会来回跳。这是卡-02 明列的禁止行为。
2. **协方差全 0**：EKF 会当成"完美测量"，输出直接跟着 IMU 走，看起来"很平滑"其实是假平滑（对应卡-02 红队提问"哪些只是看起来实现了"）。
3. **零偏没标**：静止 60 s 位姿就飘出去几米——最能验出这一条的测试是 6.2 的"零速度静止 60 s"。
4. **IMU 朝向/位置没标**：yaw 差 90° 时，转弯会往错误方向修正。
5. **`print_diagnostics` 别常开**：调优期开着看协方差，稳定后关掉，省 CPU（这台 RK3588 已经满载）。
6. **加速度进融合**：2D 轮式车融合加速度 = 主动引入漂移，别开。

以下 5 条是**本次实施真的踩到的**，写下来省下次的时间：

7. **I2C 字节序是小端**：ICM-42670 的基址寄存器放**低字节**。用大端解包会得到
   温度 −33℃、加速度模长 2.4 g 这种明显离谱的值。厂商 C 示例"看着对"是因为它
   `*(int16_t*)` 强转，在小端机上恰好等价——照抄逻辑到 Python 就会翻车。
8. **陀螺上电要等 ~0.2 s**：init 完立刻读，陀螺三轴恒为 0（加速度计正常，所以很容易误判成"陀螺坏了"）。
9. **launch 传负数值必须用 `--opt=值` 形式**：`ExecuteProcess` 把每个 cmd 元素当**一个** argv，
   所以 `['--gyro-bias', '-0.008,-0.007,0.005']` 会被 argparse 当成选项，报 `expected one argument`。
10. **`/odom` 断流时 EKF 不报错，只是协方差一路涨**：本次实测协方差从 312 涨到 5.93e4
    （std ≈ 243 m）却**没有任何 error/warn**。这正是卡-02 红队提问里"哪里会悄无声息地失败"的答案——
    **判断 EKF 是否健康，看 `/odometry/filtered` 的协方差不多于看话题频率**。
11. **`base_driver` 串口"通但下位机不应答"时是静默的**：`chassis_node.py` 里
    `vals = self._parse_ints(data); if len(vals) < 2: return` —— 一条日志都不打，
    现象就是"节点说就绪、串口已打开，但 `/odom` 一条都没有"（本次 §10 就是）。
    建议后续补一条"连续 N 秒无有效 odom → 告警"的看门狗（属卡-01/底盘范畴，不在本卡改）。
12. **`auto_drive.py drive straight -d 5` 的返程段原本停不下来**（2026-10-16 实测发现并修复）：
    `drive()` 里 `target = abs(dist)`，但 `prog`（沿初始朝向位移）是**带符号**的，
    倒车时从 0 → −5，于是 `err = target - prog = 5-(−5) = 10` **越跑越大**，
    退出条件永远不成立 → 车一直后退到超时（0.15 m/s 时 ≈105 s）才 `Abort`。
    前进段不受影响（prog 为正），所以很容易误判成"只有倒车有问题"。
    修法：`err = target - abs(prog)`（同一文件的 `turn()` 早就写的 `abs(acc)`，只有 `drive()` 漏了）。
    → 这条直接卡住验收第 1 条（`drive straight -d 5` 就是验收工具），修了才能跑。
13. **小车"物理上走不直"，开环直行的横向偏移不能当里程计精度判据**（2026-10-16）：
    4 次 `drive -p f5` 横向偏 −0.99 / −0.03 / +0.60 / +0.42 m，Δyaw −12° / +2° / +14.9° / +11.2°。
    横向 ≈ L·Δyaw/2 完全吻合 → **全部由航向漂移造成**；用户确认车物理上真的偏了 0.5 m。
    换算：**左右轮速差 ≈ 1.7%**（12°/5 m ⇒ 两轮路程差 0.209 rad × 0.408 m = 0.085 m ÷ 5 m）。
    → ① 这不是 EKF 的锅，融合陀螺也修正不了（车确实在那儿）；
      ② 已在 `auto_drive.py` 的 `drive()` 加**航向闭环**；
      ③ 1.7% 轮速差建议在 `base_driver` 里标定补偿（卡-01 范畴，未做）。
    ⚠️ 关键细节：**底盘 ω 有死区（≈0.15 rad/s）**，而抵消 1.7% 轮速差只需 0.006 rad/s，
       线性 P 直接发这么小的值底盘**根本不转**。所以用了「欠账 + 最小脉冲」调制
       （delta-sigma）：把期望的平均 ω 转成一串 ±0.20 rad/s 的脉冲，平均等效且都在死区之上。
       离线仿真（含扰动+死区，20 Hz 节流）：横向 **7.8 cm → 0.3 cm**，航向漂 4.70° → 0.09°。
       `--no-heading` 可关掉用于 A/B 对比。
    **实车复测（同日）**：`drive -p f5` 跑 3 次，横向 **+0.004 / +0.002 / +0.002 m**
       （40 cm 级 → 毫米级，比仿真预测的 ~1 cm 还好），Δyaw +0.2°/+0.8°/+0.5°，
       三次一致 → 可复现，不是运气。
       ⚠️ 另一个坑：`chassis_node` **自己就有死区补偿**（`wz_deadband=0.15`，
        `|ω|<0.15` 抬到 `wz_boost=0.18`）。所以航向闭环**绝不能**用线性 P 直接发小 ω——
        抵消 1.7% 轮速差只需 0.006 rad/s，会被底盘**放大 30 倍**抬到 0.18 → 左右振荡。
        必须用"幅值在死区之上（0.20）+ 占空比"的方式，脉冲不被抬升、幅值才可控。
14. **原地自旋的转角误差是"随机打滑"，标定救不了，只能靠陀螺**（2026-10-16 实测）：
    同配置跑两次 `drive spin`：轮速都报 ≈359.5°，陀螺（独立传感器，= 真实转角）报
    **351.5° / 357.8°** → 误差 8.5° / 2.2°，**同配置波动 6.3°**。
    ⚠️ **结论一度走错并已纠正**：先是按单次测量判为"2.27% 系统标度误差"，
       建议改 `wheel_separation` 0.4085→0.4178；既然误差是**随机**的，
       改常数只能平移均值、修不掉 ±3° 波动 → **`wheel_separation` 保持 0.4085 不动**。
       （教训：随机误差不能拿单次测量去标定，必须先跑多次看分布。）
    ✅ 正解：陀螺不受打滑影响。但当前 `odom0_config` **第 6 位（yaw）= true**，
       即 EKF 把轮速的**绝对 yaw** 当测量值 → 航向被轮速锚定，**只是复述它的错误**
       （实证：EKF Δyaw 0.04° vs /odom 0.06°，两个一样错）。
       改法：`odom0_config` 第 6 位 → **false**，yaw 改由 imu0 的 vyaw 积分得到；
       位置仍由 odom 的绝对 x,y 提供（不受影响）。
       代价：yaw 会随陀螺零偏缓慢漂移（实测零偏 ~0.04°/s），靠 SLAM 的 map→odom 兜。
       验证：改后 `drive spin`，EKF 的 Δyaw 应与陀螺一致（≈−2°），而 /odom 仍报 ≈0。
    ⚠️ 这动了 §5 已定版的融合轴，**待用户拍板，未改**。
15. **EKF 的动态响应必须拿真车验，别信仿真**：2026-10-15 在底盘没上电的情况下，用
    合成的 `/odom` + `/imu/data` 试过一轮（脚本在 `/tmp/sim*.py`，未纳入仓库），现象是
    **odom 恒定报 yaw=1.0，EKF 只到 0.5；恒定 x=1.0 也只到 0.5**，表现为收敛极慢
    （时间常数 ~7 s），像是过程噪声 Q 偏小、稳态增益太低。
    但调 Q 的结果**非单调**（Q=1 反而更差、Q=100 才变好），且同一配置多次跑结果在
    0.44~0.56 之间乱跳 —— **说明那个台架本身不可靠**（Python 紧循环发消息、到达时间不规律），
    **不能据此改配置**。
    → 结论：`ekf.yaml` **保持原值不动**，把"EKF 是否滞后于 `/odom`"列为真车验收必查项
    （见 §6.1 第 5 条）。真车数据出来的调参才作数，且每次改动都要进 §7。

---

## 9. 完成后回填卡-02

> ⚠️ **验收（§6）通过之前不要动卡-02** —— 卡是"执行检验卡"，填了就等于宣称验过了。

- [x] 卡-02 第一节「验收条件」4 项勾选（已在第四节逐条勾选并附实测值）
- [x] 卡-02 第四节「测试覆盖」8 行填满（**3 ✅ / 3 ◐ / 2 ☐**，未做的如实标注）
- [x] 卡-02 第三节「未实现项」已列 2 条 / 「需要人工确认」2 条已按 §4 §5 结论填写
- [x] 卡-02「改动文件清单」已更正为 `robot_bringup` 下实际路径（`launch/localization.launch.py` → `launch/ekf.launch.py`）
- [x] 卡-02 第五节「红队提问」已补：与需求差异 / 异常输入（未测）/ 打滑实测结论
- [x] 卡-02 第六节签核（按"已知情接受"，遗留 3 项写入签核结论）

---

## 10. ~~当前阻塞项：`/odom` 无数据~~（2026-10-15 实测，**2026-10-16 已解除**：底盘上电后 `/odom` 正常 20 Hz）

**现象**：`ekf:=true` 起全栈后，`/imu/data` 103 Hz、`/odometry/filtered` 正常输出、
`map→base_link` 整条链能解、TF 归属也正确 —— **但 `/odom` 实测 0.0 Hz**，EKF 拿不到轮速输入。

**定位过程**：

| 检查 | 结果 |
|---|---|
| `base_driver` 日志 | `串口已打开：/dev/ttyCH341USB0 @ 115200，电机已使能`、`base_driver 就绪` —— **一条错误都没有** |
| 裸测 `/dev/ttyCH341USB0`（`AT+MTENCODER?` ×2、`AT+MT=ON`） | **RX 全是空**（`b''`） |
| 裸测 `/dev/ttyACM0` | 一串二进制流 → 那是**雷达**（`/dev/LD14P` 的真实节点），不是底盘 |
| 结论 | **底盘下位机不应答**。串口转换器（CH9102）由 USB 供电，所以设备节点一直在，但下位机没在工作 |

**这不是软件问题**，是底盘上电/就绪状态问题。`base_driver` 遇到这种"串口通但下位机不应答"
会**静默**不发 odom（见 §8 第 11 条），所以现象具有迷惑性。

**续做步骤**（底盘上电后按序执行）：

1. 先确认下位机活了（**不要**启动任何 ROS 节点，否则串口被占用）：
   ```bash
   python3 -c "
   import serial, time
   s = serial.Serial('/dev/ttyCH341USB0', 115200, timeout=0.3)
   s.write(b'AT+MTENCODER?\r\n'); s.flush(); time.sleep(0.2)
   print('RX =', s.read(s.in_waiting or 1))    # 应为数字，不是 b''
   s.close()"
   ```
2. 再验 odom：
   ```bash
   ros2 launch robot_bringup bringup.launch.py phone:=false ekf:=true
   ros2 topic hz /odom        # 应为 ~20 Hz
   ```
3. `/odom` 有了之后：按 §1 存基线 → §6.1 跑验收 1/2 → §6.2 补八项测试表 → §6.3 加分项。
4. **最后**才按 §9 回填卡-02。

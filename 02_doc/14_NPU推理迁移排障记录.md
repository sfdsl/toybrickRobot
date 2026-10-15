# NPU 推理迁移排障记录（14 · 旧卡 → 新卡）

记录日期：2026-09-23
适用设备：Toybrick TB-RK3588SD 新卡（Ubuntu 22.04，ROS2 Humble）+ 旧卡 U 盘（58G ext4）
关联文档：《13_视觉识别实验方案》（本文完成其 **P0 阶段：NPU 用户态恢复** 的 C++ demo 部分）

---

## 0. 一句话结论

> 旧卡 YOLO11 demo 已完整迁移到新卡并跑通：**3 核 NPU 并行，单帧 37.5ms，实测 14~53 FPS**。
> 全程只有一个根因——**新卡镜像把 NPU/摄像头设备节点锁在 `root:video 660`**，已用 udev 规则 + 用户组双重固化。

---

## 1. 背景与迁移对象

| 项 | 旧卡 | 新卡（迁移后） |
|---|---|---|
| 系统 | 定制 Debian（Python 3.9，rknpu legacy 模式推断） | Ubuntu 22.04（内核 RKNPU driver **v0.8.5，DRM 模式**） |
| 推理代码 | `/data/aidemo/`（YOLO11 / YOLOv5 / MobileNet） | `~/RobotCode/01_Project/04_Vision/rk3588-atk-yolov11-2026/` |
| 系统库 librknnrt | 旧 | **1.6.0**（官方镜像自带，太旧，带不动 yolo11.rknn） |
| demo 自带 runtime | **2.3.2**（含 `lib/librknnrt.so` + `lib/librga.so`） | 随代码一并拷来，`run.sh` 自动优先加载 ✅ |

**只迁移了 YOLO11**（旧卡最近在用的，7/29 版）；YOLOv5 / MobileNet 未搬（见 §6 遗留）。

---

## 2. 迁移步骤（可复现）

```bash
# ① 挂载旧卡 U 盘（只读，密码为 toybrick 用户的 sudo 密码）
sudo mount -o ro /dev/sda1 /mnt/old

# ② 整体拷贝 demo（640M：cpp 源码 + yolo11.rknn + 自带 runtime）
cp -r /mnt/old/data/aidemo/rk3588-atk-yolov11-2026 ~/RobotCode/01_Project/04_Vision/

# ③ 运行（run.sh 自动定位二进制、加载自带 2.3.2 runtime、命令行降级）
cd ~/RobotCode/01_Project/04_Vision/rk3588-atk-yolov11-2026/yolo11/cpp
./run.sh 0        # 参数 = /dev/video 编号；无 DISPLAY 自动进命令行模式
```

---

## 3. 排障过程（三个坑）

### 坑 1：`failed to open rknpu module` —— NPU 设备打不开

- **现象**：`rknn_init fail! ret=-1`，1.6.0 / 2.3.2 两个 runtime 报错相同 → 排除模型版本问题，锁定设备层。
- **内核日志**（`dmesg | grep -i npu`）：
  ```
  RKNPU fdab0000.npu: Adding to iommu group 0
  [drm] Initialized rknpu 0.8.5 20230202 for fdab0000.npu on minor 1
  ```
  → 硬件正常，驱动注册为 **DRM 模式**（`/dev/dri/card1`），没有旧式的 `/dev/rknpu` 节点。
- **strace 定位**（板上有 strace，关键一步）：
  ```
  openat("/dev/rknpu", O_RDWR)      = -1 ENOENT   # legacy 路径，正常失败
  openat("/dev/dri/card0", O_RDWR)  = -1 EACCES   # ← 根因：权限
  openat("/dev/dri/card1", O_RDWR)  = -1 EACCES   # card1 就是 NPU
  ```
- **修复**：`sudo chmod 666 /dev/dri/card0 card1` → 模型立刻加载成功，3 个 NPU ctx 各绑一核。

### 坑 2：摄像头 `can't open camera by index` —— chmod 被 udev 重置

- `/dev/video0`（USB Camera，uvcvideo）同为 `root:video 660`，`toybrick` 用户不在 `video` 组。
- **手动 chmod 666 后又被重置回 660**（udev 重新套用规则）——手动改权限不可靠，必须走规则文件。
- **修复（双保险，重启永久生效）**：
  ```bash
  sudo usermod -aG video,render toybrick          # 重新登录后生效
  # /etc/udev/rules.d/99-dev-cam-npu.rules：
  #   KERNEL=="video*", MODE="0666"
  #   KERNEL=="card*",  SUBSYSTEM=="drm", MODE="0666"
  #   KERNEL=="renderD*", SUBSYSTEM=="drm", MODE="0666"
  sudo udevadm control --reload && sudo udevadm test $(udevadm info -q path -n /dev/video0)
  ```
  > 经验：改完规则用 `udevadm test $(udevadm info -q path -n <设备>)` 直接验证最终权限，比 `udevadm trigger` 猜时序可靠。

### 坑 3：命令行模式"没有输出" —— printf 管道全缓冲

- 修好权限后程序明明在跑，但管道抓不到 FPS 日志 → C 程序 stdout 进管道变**全缓冲**（报错走 stderr 所以之前看得见）。
- **解决**：调试时用 `stdbuf -oL -eL ./rknn_yolo11_cam ...` 强制行缓冲；交互使用无此问题。

---

## 4. 根因汇总

| 现象 | 根因 | 教训 |
|---|---|---|
| `failed to open rknpu module` | 新卡驱动是 DRM 模式且节点 660，用户不在 video 组 | "runtime 报 insmod 提示"≠驱动没装，先 strace 看它在 open 哪个节点 |
| 摄像头打不开、chmod 无效 | udev 每次重套 660 | 设备权限要写 udev 规则 + 加组，不要手动 chmod |
| runtime 版本困惑 | 系统库 1.6.0（镜像自带）≠ 模型所需 2.3.2 | RKNN 模型必须配 ≥ 转换工具链版本的 runtime；demo 自带 lib 用 `LD_LIBRARY_PATH` 优先加载，别动系统库 |

---

## 5. 验证数据（2026-09-23 实测）

```
$ stdbuf -oL ./rknn_yolo11_cam model/yolo11.rknn 0
[info] NPU ctx[0] 绑定核 0 / ctx[1] 绑定核 1 / ctx[2] 绑定核 2
YOLO11(3core) FPS: 15.4 | npu=37.8ms | processed=390
YOLO11(3core) FPS: 48.1 | npu=37.5ms | processed=480
YOLO11(3core) FPS: 14.7 | npu=37.6ms | processed=540
...
```
- 模型输入：640×640 NHWC INT8；9 输出（3 尺度 × 3 分支）
- NPU 单帧 **37.5ms**（≈27 FPS 纯推理）；端到端吞吐 14~53 FPS（受 USB 相机限制波动）
- 35s 稳定处理 660 帧，无报错

**日常使用**：
```bash
cd ~/RobotCode/01_Project/04_Vision/rk3588-atk-yolov11-2026/yolo11/cpp && ./run.sh 0
```

---

## 6. 遗留清单

- [ ] YOLOv5（`rk3588-atk-yolov5_2026`）、MobileNet demo 未迁移（旧卡 `/mnt/old/data/aidemo/` 均在，方法同 §2）
- [ ] 重新登录/重启一次，确认 `usermod` 组生效（udev 规则已独立生效，此为双保险）
- [ ] 《13》P1~P5：相机 → `/image_raw` → 推理 → `/detections` → 手机 HUD / 雷达融合 / 导航联动
- [ ] 旧卡 U 盘仍只读挂在 `/mnt/old`，用完 `sudo umount /mnt/old`

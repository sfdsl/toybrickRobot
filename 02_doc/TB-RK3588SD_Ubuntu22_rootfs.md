# TB-RK3588SD 换装 Ubuntu 22.04 rootfs 指南

## 1. 背景与目标

**适用对象**：Toybrick TB-RK3588SD，原厂系统为 Debian，RK3588 平台。

**核心目标**：仅替换 SD 卡上的 rootfs 为 Ubuntu 22.04，保留原厂 U-Boot / 内核 / dtb / 启动链不变。上板后运行 ROS2 Humble，实现雷达建图、自主导航、行为树巡检。

**为什么换 Ubuntu 22.04**：

| 理由 | 说明 |
|------|------|
| ROS2 生态匹配 | Humble 官方二进制面向 Ubuntu 22.04 Jammy，apt 直接装 `ros-humble-*` |
| 导航/SLAM 包最全 | nav2、slam_toolbox、behavior_tree、foxglove 等在 22.04 aarch64 上资料最多 |
| Debian 维护成本高 | 官方 Foxy 已 EOL，源码编译/容器方案增加适配负担 |
| 安全可控 | 换错只需重做 SD，原 Debian 系统通过拔卡恢复 |

**为什么不直接刷整卡镜像**：通用 RK3588 Ubuntu 整包会覆盖 U-Boot/内核/dtb，与你"保留原厂启动链"的目标冲突。除非你打算完全重适配所有驱动（WiFi/蓝牙/NPU/ISP/串口），否则不要走这条路。

## 2. 启动链与职责边界

TB-RK3588SD 原厂启动大致流程：

```
SPI Flash (存 uboot) → boot 分区 (后续启动内容) → SD/TF 卡 (rootfs)
```

不同批次/固件可能不同，动手前先用串口看 U-Boot 输出，在原系统执行：

```bash
uname -a              # 内核版本
lsblk                 # 各分区挂载关系
cat /proc/cmdline     # 内核命令行，看 root= 指向哪
```

**本方案分工原则**：

| 组件 | 位置 | 是否改动 |
|------|------|---------|
| U-Boot | SPI Flash | ❌ 不动 |
| SPL / Miniloader | SPI / eMMC | ❌ 不动 |
| 内核 Image | boot 分区 | ❌ 不动 |
| dtb | boot 分区 / SPI / FIT | ❌ 不动 |
| 内核模块 | rootfs `/lib/modules/$(uname -r)` | ✅ 从原 Debian 拷入 |
| 固件 | rootfs `/lib/firmware` | ✅ 从原 Debian 拷入 |
| 用户态系统 | SD rootfs | ✅ 替换为 Ubuntu 22.04 |

> **关键约束**：新 rootfs 的内核模块版本必须与原厂运行内核版本完全一致。

## 2.5 新系统实况核对（2026-09-22 换卡**后**实测 ✅）

> **换卡已完成**：当前这张 SD 卡跑的就是 **Ubuntu 22.04.5 + 原厂 5.10.160 内核**，已正常启动联网。
> 下表是**换卡后**在板子上实测出来的（不是推测），后续所有工作以它为准。
> 旧 Debian 卡**保持原样不动**，随时拔卡换回即可回退。

| 项 | 实测值 | 状态 / 说明 |
|---|---|---|
| 系统 / hostname | Ubuntu 22.04.5 LTS (jammy) / `toybrick-ubuntu` | ✅ 目标达成 |
| 内核 | `5.10.160`（`Linux toybrick-ubuntu 5.10.160 #6 SMP ... aarch64`） | ✅ 与原厂一致，启动链未动 |
| 根分区 | **`/dev/mmcblk1p1`**（58.2 G ext4，UUID `159ead16-1182-4d87-8938-812d6a33fdb0`，单分区占满） | ✅ 实测单分区可启动（见 §2.5.1） |
| cmdline | 仍写 `root=/dev/mmcblk1p2` | ⚠ 与实际 p1 不符，但**实测能起来** |
| 磁盘余量 | 58 G 用 2.7 G，**剩 52 G** | 装 ROS2 + nav2（约 8~12 G）绰绰有余 |
| 内核模块 | `/lib/modules` 下**只有 `5.10.160`** | ✅ 没带进 x86 残留 `4.15.0-201-generic` |
| 固件 | `/lib/firmware` 32 M（BCM/蓝牙 .hcd 齐全） | ✅ WiFi 已用上 |
| 网络 | `wlan0` = `192.168.31.73/24`，有网 | ✅ 固件 + NetworkManager/wpa 生效 |
| apt 源 | 清华 `ubuntu-ports` jammy{,-updates,-security} | ✅ |
| udev 别名 | `/dev/LD14P → ttyACM0`、`/dev/CAR_BASE → ttyCH341USB0`、`/dev/i2c-6`（root:dialout 0660） | ✅ 6 条规则生效，另增 `99-robot-i2c.rules` |
| 账号 | `toybrick`（uid 1000）在 `dialout` + `sudo` | ✅ 别名可访问 |
| 工程文件 | `~/RobotCode` 161 M 已在位：`04_map/`（robot_map.pgm/.yaml/.posegraph）、`logs/`、`ros2_ws/src/{base_driver, ldlidar, robot_bringup, slam_toolbox}`、`start_robot.sh` | ✅ |
| `.bash_aliases` | 已全部指向 `/opt/ros/humble/setup.bash`（10 处） | ✅ |
| Foxy 残留 | 工程代码里 `ros2-foxy` **0 处**（343 处全在 `02_doc` 历史 JSON/文档里，不用改） | ✅ |
| **ROS2** | **`/opt/ros` 不存在 → 还没装** | ⬜ 下一步（§7） |
| **Toybrick 专有包** | `dpkg -l \| grep toybrick` → **0 个** | ⬜ 待办，见 §2.5.2 |
| NPU / 多媒体节点 | `/dev/dri/{card0,card1,renderD128,129}`、`/dev/mpp_service`、`/dev/rga` **均在** | 内核驱动 OK，**用户态库缺**（§2.5.2） |

### 2.5.1 "cmdline 写 p2、实际跑 p1"的答案（已实测）

之前担心的"U-Boot 到底按什么找根分区"现在有结论：**单分区 p1 占满整卡可以正常启动**，
`root=/dev/mmcblk1p2` 与实际不符并不会导致起不来（U-Boot/内核会 fallback 到可启动分区）。

→ **新卡就照现卡布局做**（单分区 ext4 + `fstab` 用 UUID），**不需要为了 p2 去建两个分区**。

### 2.5.2 换卡后才看得出来的两个缺口

| # | 缺口 | 影响 | 处理 |
|---|---|---|---|
| 1 | 7 个 Toybrick 专有包**一个都没装**（`python3-toybrick`、`toybrick-prop(-bin)`、`toybrick-server`、`toybrick-usbd`、`toybrick-vendor(-bin)`） | WiFi/蓝牙/电源管理**已实测正常**（驱动+固件到位），但 **NPU(RKNN) / RGA / MPP 的用户态库与 `toybrick-server` 缺失** → 只要不跑 NPU 推理就不影响 ROS 导航 | 暂不急；真要用 NPU 时按 §4 从旧卡 `dpkg -L` 搬文件回来 |
| 2 | ROS2 Humble 未安装 | 所有 `robotnav`/`navcheck`/`mapv` 等命令都跑不了 | 直接进 §7，apt 装即可（22.04 上比 Foxy 源码编译省事得多） |

### 开工前第一件事：确认 U-Boot 怎么找 rootfs（✅ 已由实测结案，仅存档）

当时的问题：Debian 上 `cmdline` 写 `root=/dev/mmcblk1p2`，实际却跑在 `mmcblk1p1` 上。

**结论（2026-09-22 换卡后实测）**：照现卡做成**单分区 p1 + fstab 用 UUID**，插上就能起，
不会"内核起来了找不到根分区"。所以下面这些排查命令**只在需要重做卡时才跑**：

```bash
# ① 现系统上（只读）
cat /proc/cmdline
cat /etc/fstab                 # / 是按设备名还是 UUID 挂的
lsblk -f ; blkid

# ② 串口接 U-Boot：上电，倒计时结束前按键停下，然后
printenv bootargs
printenv bootcmd
part list mmc 1                # U-Boot 眼里的 SD 分区表
```

> **稳妥做法**：新卡先做成**与现卡完全一致的布局**（单分区 ext4 占满 + `fstab` 与现卡同样的写法）。
> 若 U-Boot 的 `bootargs` 里硬写了 `root=/dev/mmcblk1p2`，那就**把 rootfs 放在第 2 分区**。

## 2.6 制作路线怎么选（先决定，再动手）

| | **方案 A：板子上直接做（native，推荐）** | **方案 B：x86 主机 + qemu（§3 原流程）** |
|---|---|---|
| 前提 | 板子能装 `debootstrap`（Debian 源可用 ✅，实测候选版本 `1.0.123+deb11u2`）；**需要一个 USB 读卡器**把新卡插到板子上 | 一台 x86 Ubuntu（或 WSL2）+ 能下 `ubuntu-base` tar |
| 同构性 | arm64 → arm64，**不需要 qemu**，没有 binfmt 那类坑 | 需要 `qemu-user-static` + binfmt |
| 速度 | 原生执行，快 | 模拟执行，慢 |
| 后续 | **§4 之后完全一样** | §3.1~3.5 |

**方案 A 流程（2026-09-22 已在板子上实跑验证）**——推荐"**先做成目录，最后再 rsync 到卡**"，
这样不用等卡、不怕格错，出错重来也便宜：

```bash
# ① 下载 arm64 基础包（约 26 MB，清华镜像，实测 200）
mkdir -p ~/tb3588 && cd ~/tb3588
wget -c https://mirrors.tuna.tsinghua.edu.cn/ubuntu-cdimage/ubuntu-base/releases/22.04/release/ubuntu-base-22.04.5-base-arm64.tar.gz

# ② 原生解包（arm64→arm64，**不需要 qemu**）
sudo mkdir -p ~/tb3588/rootfs
sudo tar -xzf ubuntu-base-22.04.5-base-arm64.tar.gz -C ~/tb3588/rootfs
file ~/tb3588/rootfs/bin/bash        # 应显示 "ARM aarch64" → 说明同构，后续全部原生执行

# ③ 写 apt 源（换清华，快很多）+ DNS + hostname
#    /etc/apt/sources.list → http://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports/ jammy{,-updates,-security}
#    /etc/resolv.conf      → nameserver 223.5.5.5
#    /etc/hostname         → toybrick-ubuntu

# ④ 放入驱动包（⚠ 必须带 --keep-directory-symlink，见下方坑）
sudo tar --keep-directory-symlink -xzf /tmp/toybrick_drivers.tar.gz -C ~/tb3588/rootfs

# ⑤ chroot 装基础包（原生 chroot，秒开）
sudo mount -t proc proc ~/tb3588/rootfs/proc
sudo mount -t sysfs sys ~/tb3588/rootfs/sys
sudo mount -o bind /dev ~/tb3588/rootfs/dev
sudo chroot ~/tb3588/rootfs /bin/bash
#   里面执行：apt-get update && apt-get install -y <§3.4 清单 + network-manager wpasupplicant>

# ⑥ 装完 rsync 到新卡（§5.2）+ 写 fstab（§5.3）
```

> **⚠ 实测踩到的坑（务必注意）**：`toybrick_drivers.tar.gz` 里的路径是 `lib/modules/...`、`lib/firmware`，
> 直接解包会把 Ubuntu 合并式布局的符号链接 **`/lib → usr/lib` 覆盖成真目录**，于是
> `/lib/ld-linux-aarch64.so.1` 断链 → **`chroot` 报 `failed to run command '/bin/bash': No such file or directory`**
> （症状看着像"bash 不存在"，其实是动态解释器找不到）。两种修法：
>
> ```bash
> # A) 解包时就保留符号链接（推荐，一步到位）
> sudo tar --keep-directory-symlink -xzf /tmp/toybrick_drivers.tar.gz -C ~/tb3588/rootfs
>
> # B) 已经解错了 → 把内容挪到 usr/lib，再恢复 /lib 链接
> sudo mv ~/tb3588/rootfs/lib/modules  ~/tb3588/rootfs/usr/lib/
> sudo mv ~/tb3588/rootfs/lib/firmware ~/tb3588/rootfs/usr/lib/
> sudo rmdir ~/tb3588/rootfs/lib && sudo ln -s usr/lib ~/tb3588/rootfs/lib
> ```

> 备选路线：`debootstrap`（`sudo apt install debootstrap` 后
> `sudo debootstrap --arch=arm64 jammy <目标目录> http://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports/`）。
> 注意 **Debian 11 自带的 debootstrap 可能不认识 `jammy` 这个 suite**（脚本/密钥环缺失），
> 所以本次实测走的是上面的 ubuntu-base 路线（更短、更稳）。

> 板内路线**没有"下载 ubuntu-base tar"这一步**（debootstrap 直接按 jammy 的包列表装）。
> 如果没有读卡器：也可以在板内 `debootstrap` 到 `~/tb3588/rootfs`（41 G 够用），再 `tar` 出来拷到 x86 主机写卡（等同方案 B 的写卡步骤）。

### 「板子上直接做」是什么意思（人话版）

换系统 = **准备一整棵 Ubuntu 的文件树**（`/bin`、`/etc`、`/usr`…），不是点鼠标"安装"。
由谁来做这棵树，取决于 **CPU 架构能不能直接执行这些程序**：

- 板子本身是 **arm64**，Ubuntu 22.04 的软件包也是 **arm64** → **同一种架构，能直接跑**。
  所以可以让**板子自己**把 Ubuntu 装进「外接的那张新卡」里 —— 这就是方案 A，
  **native（原生）= 不经过翻译/模拟**。
- **x86 电脑**只会执行 x86 程序，处理 arm64 的包必须请"翻译"（`qemu-aarch64` + binfmt）——
  这就是方案 B，慢，而且容易冒 `Exec format error` 那类怪问题。

```text
方案 A（板内，native）                    方案 B（x86 主机 + qemu）
  ┌────────────┐                          ┌──────────────┐
  │ 板子 arm64 │ ← 直接执行 arm64 的包     │  x86 电脑    │ ← 只能执行 x86
  │  （旧卡）  │                          │  + qemu 翻译 │   → 模拟 arm64
  └─────┬──────┘                          └──────┬───────┘
        │ USB 读卡器                             │ 读卡器 / SD 槽
  ┌─────▼──────┐                          ┌──────▼───────┐
  │ 新 SD 卡   │ ← debootstrap 直接写入     │ 新 SD 卡     │ ← 拷入 rootfs
  └────────────┘                          └──────────────┘
```

> **两个关键认识**（很多人卡在这）：
> 1. 方案 A **不是"给板子自己换系统"**，而是"给**外接的读卡器里那张新卡**准备系统"。
>    板子自己的系统卡是 `/dev/mmcblk1`，**全程一个字节都不动**（旧卡就是安全网）。
> 2. 方案 A 唯一的额外硬件需求：**一个 USB 读卡器**，把新 SD 卡插到板子的 USB 口上。

> **插上新卡后，先确认它是哪个设备（别格错卡！）**：
> - 板子自己的系统卡永远是 **`/dev/mmcblk1`**（挂在 mmc 总线上，58 G 那个）
> - 读卡器走 USB 总线 → 新卡显示成 **`/dev/sda`（或 `sdb`…）**
> - 动手前先跑一次看清楚：
>
> ```bash
> lsblk -o NAME,SIZE,TYPE,MODEL,MOUNTPOINT
> # 例：
> #   mmcblk1      58.2G  disk            ← 板子自己的系统卡（绝对不要动）
> #   └─mmcblk1p1  58.2G  part  /         ← 现在跑着的系统
> #   sda          59.5G  disk  读卡器     ← 刚插的新卡，要写的就是它
> #   └─sda1                 （还没分区）
> ```

## 3. 在 x86 主机制作 arm64 rootfs（方案 B，需 qemu）

### 3.1 环境准备

物理 Ubuntu x86 或 WSL2 均可。WSL2 的 binfmt 偶尔异常，物理机更稳。

```bash
sudo apt update
sudo apt install -y qemu-user-static binfmt-support debootstrap \
  rsync squashfs-tools git
```

注册 aarch64 binfmt 解释器（**不注册会在 chroot 时报 Exec format error**）：

```bash
sudo update-binfmts --enable qemu-aarch64
cat /proc/sys/fs/binfmt_misc/qemu-aarch64
# 应输出包含 interpreter /usr/bin/qemu-aarch64-static 的内容
```

### 3.2 下载 ubuntu-base

官方目录：

```
https://cdimage.ubuntu.com/ubuntu-base/releases/22.04/release/
```

下载 arm64 基础包：

```bash
wget https://cdimage.ubuntu.com/ubuntu-base/releases/22.04/release/ubuntu-base-22.04.5-base-arm64.tar.gz
```

国内镜像加速（替换域名即可）：

- `https://mirrors.bfsu.edu.cn/ubuntu-cdimage/ubuntu-base/releases/22.04/release/`
- `https://mirrors.aliyun.com/ubuntu-cdimage/ubuntu-base/releases/22.04/release/`
- `https://mirrors.ustc.edu.cn/ubuntu-cdimage/ubuntu-base/releases/22.04/release/`

### 3.3 解包与注入 qemu

```bash
mkdir -p ~/tb3588/rootfs
tar -xzf ubuntu-base-22.04.5-base-arm64.tar.gz -C ~/tb3588/rootfs
cp /usr/bin/qemu-aarch64-static ~/tb3588/rootfs/usr/bin/
```

### 3.4 chroot 初始化基础系统

```bash
cat > ~/tb3588/chroot_init.sh <<'EOF'
#!/bin/bash
R=~/tb3588/rootfs

# 挂载虚拟文件系统
mount -t proc proc $R/proc
mount -t sysfs sys $R/sys
mount -o bind /dev $R/dev
mount -o bind /dev/pts $R/dev/pts

# chroot 内安装基础包
chroot $R /bin/bash -c '
export DEBIAN_FRONTEND=noninteractive
apt update || true
apt install -y locales tzdata sudo openssh-server systemd udev dbus \
  netplan.io ifupdown iproute2 resolvconf psmisc vim nano \
  ca-certificates wget curl gnupg lsb-release htop i2c-tools \
  python3 python3-pip python3-numpy python3-opencv python3-serial \
  rsync usbutils pciutils net-tools
locale-gen en_US.UTF-8
ln -sf /usr/share/zoneinfo/Asia/Shanghai /etc/localtime
'

# 卸载
umount -R $R/proc 2>/dev/null
umount -R $R/sys 2>/dev/null
umount $R/dev/pts 2>/dev/null
umount $R/dev 2>/dev/null
EOF

bash ~/tb3588/chroot_init.sh
```

### 3.5 使用 carrot-ubuntu 脚本（备选）

如果你已用 carrot-ubuntu 脚本：

```bash
cd carrot-ubuntu
sudo ./make_ubuntu_rootfs.sh
# 选 1) 22.04/Jammy
# 选 2) server
```

**前置条件**：确保 `qemu-user-static` + `binfmt-support` 已装，否则报 `Exec format error`。

**注意**：脚本产物若含内核/boot/dtb，按"原厂启动链优先"原则剔除，只取 rootfs 内容。

## 4. 迁移驱动、固件、规则与专有包（本机清单）

在原 Debian 系统上执行，打包后解到新 rootfs：

```bash
# ===== 原系统执行 =====
KVER=$(uname -r)                  # 本机 = 5.10.160
OUT=/tmp/oldroot
mkdir -p $OUT/lib $OUT/etc

# ① 内核模块：**只拷当前内核这一个目录**
#    ⚠ 本机 /lib/modules 下还有个 x86 残留 4.15.0-201-generic，千万别一起带走
cp -a /lib/modules/$KVER $OUT/lib/modules/

# ② 固件（WiFi / 蓝牙 / ISP …）
cp -a /lib/firmware $OUT/lib/

# ③ udev 规则：整目录拷（本机 99-robot.rules 有 6 条串口别名，见 §5.4）
mkdir -p $OUT/etc/udev
cp -a /etc/udev/rules.d $OUT/etc/udev/rules.d.orig

# ④ 网络（WiFi 连接信息在这里）
cp -a /etc/network $OUT/etc/ 2>/dev/null
cp -a /etc/netplan $OUT/etc/ 2>/dev/null
cp -a /etc/wpa_supplicant $OUT/etc/ 2>/dev/null

# ⑤ 账号/组信息（备查：新系统建号时让 uid/gid 与现系统一致，省得改 udev 里的组名）
grep -E '^(toybrick|root|dialout|i2c)' /etc/passwd /etc/group > $OUT/etc/account_groups.txt

# ⑥ 打包
tar -czf /tmp/toybrick_drivers.tar.gz -C $OUT lib etc
```

拷到新 rootfs（写卡前解压）：

```bash
sudo tar -xzf toybrick_drivers.tar.gz -C /mnt/newroot/     # 方案 A
# 或：sudo tar -xzf toybrick_drivers.tar.gz -C ~/tb3588/rootfs/   # 方案 B
```

**Toybrick 专有包（本机实测 7 个，必须逐个搬）**：

```bash
dpkg -l | awk '/toybrick/{print $2}'
# → python3-toybrick / toybrick-prop / toybrick-prop-bin /
#   toybrick-server / toybrick-usbd / toybrick-vendor / toybrick-vendor-bin

# 这些包只提供用户态文件（不碰内核），所以"按文件清单搬运"可行：
for p in python3-toybrick toybrick-prop toybrick-prop-bin toybrick-server \
         toybrick-usbd toybrick-vendor toybrick-vendor-bin; do
  echo "== $p =="; dpkg -L $p | grep -vE '^/(usr/share/(doc|man)|var/lib/dpkg)' | head -30
done > /tmp/toybrick_pkg_files.txt
```

> 经验：这几个包管的是 **WiFi/蓝牙、电源管理（prop）、NPU/MPP/RGA 用户态**——
> 漏拷的典型症状就是 §6 排查表里的"能开机但没网络"。

**另外别忘了带走你自己的东西**（不在上面这份 tarball 里）：

```bash
tar -czf /tmp/robotcode.tar.gz -C ~ RobotCode      # 含 04_map 地图、logs、工具、文档
cp ~/.bash_aliases /tmp/                            # 你的别名（23 处引用 ros2-foxy，见 §7）
# 强烈建议：现卡整盘克隆一份（见 11_备份与恢复.md）→ 出问题拔卡即回退
```

> ⚠ **换系统后所有 ROS 路径都要改**：`.bash_aliases` 里 **23 处** `ros2-foxy`、
> `start_robot.sh`、launch/config、脚本文档里的 `/opt/ros/humble/setup.bash` ——
> 详见 §7「Foxy → Humble 迁移清单」。

## 5. SD 卡制作

### 5.1 分区

SD 卡用 GPT 分区表，根分区 ext4。分区大小建议 30-40GB（64GB SD 可用约 58-60GB）。

> ⚠ **本方案优先级最高的一条**：现卡（Debian）实测是 **单分区 `mmcblk1p1` 占满整卡**，
> 但 `/proc/cmdline` 里写的是 **`root=/dev/mmcblk1p2`** —— 两者不一致。写卡前必须先按
> §2.6 上方"开工前第一件事"确认 U-Boot 到底按什么找根分区：
>
> - 按 **UUID** 或"第一个 ext4 分区"找 → **单分区**即可（与现卡一致，最稳）
> - 若 `bootargs` 里**硬写了 `root=/dev/mmcblk1p2`** → **必须建两个分区**，rootfs 放进 **p2**
>
> 第一版建议**照现卡做**（单分区 + rootfs 在 p1），能起来再谈优化。

```bash
# 假设 SD 设备为 /dev/sdX，请确认后再操作！
sudo parted /dev/sdX mklabel gpt
sudo parted /dev/sdX mkpart primary ext4 0% 100%
sudo mkfs.ext4 /dev/sdX1
```

**若确认必须用 p2 放 rootfs**：

```bash
sudo parted /dev/sdX mklabel gpt
sudo parted /dev/sdX mkpart primary ext4 0% 1%        # p1：预留
sudo parted /dev/sdX mkpart primary ext4 1% 100%      # p2：rootfs
sudo mkfs.ext4 /dev/sdX2
```

### 5.2 写入 rootfs

```bash
sudo mkdir -p /mnt/sdroot
sudo mount /dev/sdX1 /mnt/sdroot

# 用 rsync 保留权限
sudo rsync -axHAX --numeric-ids ~/tb3588/rootfs/ /mnt/sdroot/

sudo umount /mnt/sdroot
```

### 5.3 配置 fstab

```bash
sudo mount /dev/sdX1 /mnt/sdroot
blkid /dev/sdX1   # 获取 UUID
```

编辑 `/mnt/sdroot/etc/fstab`：

```
UUID=xxxx-xxxx / ext4 defaults,noatime 0 1
```

> 不要挂载不存在的 `/boot/efi`，除非你确认启动链需要。

### 5.4 串口与 I2C 固定名（**直接沿用现卡规则，不要重写**）

现卡规则文件 `/etc/udev/rules.d/99-robot.rules`（已在 §4 ③ 打包）内容如下，**原样复制**——
本机实测设备是 **LD14P → `ttyACM0`（`1a86:55d4`）**、**底盘 → `ttyCH341USB0`（`1a86:7523`）**：

```
# LD14P 雷达（QinHeng USB Single Serial）
KERNEL=="ttyACM*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d4", MODE:="0777", GROUP:="dialout", SYMLINK+="LD14P"
KERNEL=="ttyCH343USB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d4", MODE:="0777", GROUP:="dialout", SYMLINK+="LD14P"
KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", ATTRS{serial}=="0001", MODE:="0777", GROUP:="dialout", SYMLINK+="LD14P"

# 底盘串口（CH341）
KERNEL=="ttyCH341USB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", MODE:="0777", GROUP:="dialout", SYMLINK+="CAR_BASE"
KERNEL=="ttyUSB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", MODE:="0777", GROUP:="dialout", SYMLINK+="CAR_BASE"
KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", ATTRS{serial}!="0001", MODE:="0777", GROUP:="dialout", SYMLINK+="CAR_BASE"
```

**顺带补上 IMU 那一行**（现卡也还没有，见 08 方案 N6；没有它就要 `sudo` 才能读 `/dev/i2c-6`）：

```
# 电源板 IMU（I2C-6 / 0x69）
KERNEL=="i2c-6", MODE="0660", GROUP="dialout"
```

应用与验证：

```bash
# 写卡阶段：直接放进 /mnt/sdroot/etc/udev/rules.d/99-robot.rules
sudo udevadm control --reload-rules && sudo udevadm trigger   # 上板后
ls -l /dev/LD14P /dev/CAR_BASE        # 应指向 ttyACM0 / ttyCH341USB0
ls -l /dev/i2c-6                      # 应为 crw-rw---- root dialout
id toybrick                           # 必须在 dialout 组里（否则别名不可访问）
```

### 5.5 账号与 SSH

chroot 或上板后：

```bash
# 建用户
adduser robot
usermod -aG sudo robot

# 允许 SSH root 登录（调试阶段，正式部署建议禁用）
sed -i 's/#PermitRootLogin prohibit-password/PermitRootLogin yes/' /etc/ssh/sshd_config
systemctl enable ssh
```

卸载：

```bash
sudo umount /mnt/sdroot
```

## 6. 上板首次启动核查

插 SD 卡，从原 Debian 正常关机后插入 TF 卡，再上电（TB-RK3588SD 资料提示"预装 rootfs 的 TF 卡不支持热插拔"）。

串口看 U-Boot 输出，确认仍按原厂加载内核/dtb。

进系统后执行：

```bash
uname -a                        # 内核版本与原厂一致
ls /lib/modules/$(uname -r)     # 非空
find /lib/firmware -maxdepth 1 | head   # 含原厂固件
systemctl status systemd-udevd ssh      # 正常
ip a                             # 网络
ls /dev/ttyUSB*                  # 串口识别
ls -l /dev/lidar /dev/chassis    # udev 别名
dmesg | egrep -i 'usb|lidar|motor|panic|error'
```

**排查清单**：

| 现象 | 可能原因 | 解决 |
|------|---------|------|
| 未从 SD 启动 | fstab UUID 错误/分区格式不对 | 检查 blkid 与 fstab |
| 无网络 | modules/firmware 未拷全 | 重新拷 /lib/modules 和 /lib/firmware |
| 串口不识别 | udev 规则未生效 | udevadm control --reload && trigger |
| 内核模块报错 | 版本不匹配 | 确认 uname -r 与 modules 目录名一致 |

## 7. ROS2 Humble 安装

```bash
sudo apt update
sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update

# 添加 ROS2 apt 源
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
| sudo tee /etc/apt/sources.list.d/ros2.list

sudo apt update

# 安装 ros-base（不装 desktop，车端不需要图形）
sudo apt install -y ros-humble-ros-base

# 导航与 SLAM
sudo apt install -y ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-slam-toolbox ros-humble-map-server \
  ros-humble-robot-localization

# 雷达驱动：本机是 LD14P（不是 RPLIDAR）→ apt 里没有，用自己编译的
#   ❌ 不要装 ros-humble-rplidar-ros（那是 RPLIDAR 的驱动）
#   ✅ 用 ~/RobotCode/ros2_ws/src/ldlidar（从现系统带过来，见 §7.1）

# 差速控制器 + 图像桥（camera_lidar_view 可能用到）
sudo apt install -y ros-humble-diff-drive-controller \
  ros-humble-controller-manager \
  ros-humble-cv-bridge ros-humble-image-transport

# 构建工具 + 依赖安装器
sudo apt install -y python3-colcon-common-extensions python3-rosdep
```

**空间占用参考**（apt 安装后）：

| 组件 | 大小 |
|------|------|
| Ubuntu 22.04 server base | 2-5 GB |
| ros-humble-ros-base | 1-2 GB |
| nav2 + slam_toolbox + 依赖 | 2-4 GB |
| 雷达/底盘驱动 | < 100 MB |
| **合计** | **约 8-12 GB** |

64GB SD 装完留 40GB+ 空闲，足够日志、rosbag、地图。

### 7.1 Foxy → Humble 迁移清单（本机专用，**别漏**）

| 要改的东西 | 现状 | 改成 |
|---|---|---|
| `~/.bash_aliases` | ✅ 已改完：10 处 `source /opt/ros/humble/setup.bash`（原 23 处 foxy） | `/opt/ros/humble/setup.bash` |
| `start_robot.sh` | 硬编码 `/opt/ros/humble/setup.bash`（另有 `set -u` 下 source 的坑，已修过） | 同样替换；建议把 ROS 路径提成一个变量 |
| launch / config / 脚本 | `ros2_ws/src/*/launch/*.py` 等可能含绝对路径 | `grep -rn ros2-foxy ~/RobotCode` 逐个清 |
| 文档 | 02_doc 多处路径与版本描述 | 顺手更新 07 / 08 / 09 |

```bash
# 上板后一次性找全；改完复查应为 0 条
grep -rn 'ros2-foxy' ~/RobotCode ~/.bash_aliases 2>/dev/null | wc -l
```

**工作区重编（已实测跑通，2026-09-22）**：

```bash
source /opt/ros/humble/setup.bash
cd ~/RobotCode/ros2_ws
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble
# ⚠ 两个必需参数：
#   --base-paths src          → 否则会把 patches/ 也当包扫进来，报"Duplicate package names"
#   --allow-overriding slam_toolbox → 源码 fork 版与 apt 版同名，必须显式允许覆盖
colcon build --base-paths src --symlink-install --parallel-workers 4 --allow-overriding slam_toolbox
source install/setup.bash
```

结果：`base_driver` / `ldlidar` / `slam_toolbox`(2min22s) / `robot_bringup` **4 个全部编译通过** ✅

**fork 版 slam_toolbox 的 Foxy→Humble 适配改动（已 `sync_patches.sh` 存补丁，12 个文件）**：

| 文件 | 改动 | 原因 |
|---|---|---|
| `CMake/FindTBB.cmake` | 版本头改为 `tbb_stddef.h` 不存在时读 `oneapi/tbb/version.h` | oneTBB 2021.5 移除了 `tbb_stddef.h` |
| `lib/karto_sdk/include/karto_sdk/Mapper.h` | `#include "tbb/parallel_do.h"` → `parallel_for_each.h` | 同上，旧头已删 |
| `lib/karto_sdk/src/Mapper.cpp` | `tbb::parallel_do(...)` → `tbb::parallel_for_each(begin, end, ...)` | oneTBB 移除 `parallel_do` |
| `src/slam_toolbox_common.cpp` | `minimum_time_interval_(0.)` → `rclcpp::Duration::from_seconds(0.)` | Humble 删除 `Duration(double)` 隐式构造 |
| 同上 | `declare_parameter("paused_new_measurements")` → 加默认值 `false` | Humble 无单参重载 |
| 同上 | `map_start_pose` / `map_start_at_dock`：改 `dynamic_typing` 声明 + `get_parameter()` + `get_value<T>()` | Humble 用 `get_value<T>()`；需保留"未设置"语义 |
| `include/slam_toolbox/visualization_utils.hpp` | `rclcpp::Duration(0.)` → `from_seconds(0.)` | 同上 |
| `slam_toolbox_{localization,sync,async,lifelong}_node.cpp` | `declare_parameter("stack_size_to_use")` → 加默认值 `stack_size` | 同上 |

> 顺带补装了 **git**（换系统后缺失，`sync_patches.sh` 依赖它）。rosdep 走 ghproxy 代理 `ROSDISTRO_INDEX_URL` 才能 update 成功。

**两个"要不要换成 apt 版"的取舍**：

- **slam_toolbox**：现系统用 `ros2_ws/src/slam_toolbox`（自己 fork：localization 模式 +
  `mapper_params_localization.yaml`，含 `map_start_pose`、`map_update_interval: 1.0`、
  `map_file_name=~/RobotCode/04_map/robot_map`）→ **先源码编译跑通旧的**，保证 N1 定位链路不变；
  想换 apt 版必须**逐项核对参数名与 launch 结构**，用 `mapv`/`navcheck` 验证过再切。
- **nav2 / robot_localization**：Humble 上 apt 直接可装 ✅（08 方案里论证过 Foxy 时代源码构建 nav2 风险高）。
  这正是换 22.04 的最大收益：nav2 的代价地图/恢复行为，以及 N6-I4 想要的 EKF 融合，都省掉构建成本。

**迁移后的验证顺序**（等于把 08 方案的 N1 重跑一遍）：

```bash
robotcheck                 # 设备别名/权限
odomhz ; scanhz            # 底盘 / 雷达活着没
robotnav                   # 定位模式起全（车摆建图起点）
mapv                       # 车标与实际是否吻合（N1 验收 ①②③）
navcheck --goal 2.0 1.5    # 离线规划判据
nav --listen               # 手机点一下，真机跑一趟
```

## 8. 常见坑与注意事项

| 坑 | 原因 | 对策 |
|---|---|---|
| chroot 报 `Exec format error` | 未注册 qemu-aarch64 binfmt | `sudo update-binfmts --enable qemu-aarch64` |
| 模块版本不匹配 | 新 rootfs 内核版本 ≠ 原厂 | `uname -r` 必须等于 `lib/modules/` 目录名 |
| SD 卡损坏 | rosbag/日志高频写 | logrotate + 限制 bag 数量 + 重要数据另存 |
| 串口名漂移 | 未写 udev 规则 | 用 idVendor/idProduct 固定别名 |
| 换 rootfs 后驱动异常 | 误带新内核到 rootfs | rootfs 不放内核/boot/dtb |
| 中文构建路径 | 脚本 chroot/mount 异常 | 工作目录用纯英文 |
| WSL2 binfmt 抽风 | WSL 的 binfmt_misc 行为异常 | 改用物理 Ubuntu 或 Docker |
| 整包镜像覆盖启动链 | 用了通用 RK3588 整卡镜像 | 只用 rootfs tar，不刷 U-Boot/内核/dtb |
| **拷了 x86 残留模块目录** | `/lib/modules` 下同时有 `5.10.160` 与 `4.15.0-201-generic` | **只拷 `5.10.160`**（§4 ①） |
| **能开机但没网络/WiFi** | 漏了 toybrick 专有包或 `/lib/firmware` | 按 §4 的 7 个包 + firmware 全量补齐 |
| **找不到根分区** | U-Boot `bootargs` 指 `mmcblk1p2`，新卡只建了 p1 | 见 §2.5「开工前第一件事」——先确认再做卡 |
| 串口别名不可用/权限不足 | 用户不在 `dialout` 组，或规则缺 `MODE:="0777"` | 沿用现规则（§5.4）+ `id toybrick` 确认在组里 |
| ROS 命令找不到 / 环境串了 | 别名或脚本还指向 `/opt/ros/humble` | §7.1：`grep -rn ros2-foxy ~/RobotCode` 应为 0 |
| slam_toolbox 起不来或参数报错 | apt 版参数名/结构与自 fork 版不同 | 先用源码版跑通；换 apt 版逐项核对（§7.1） |
| 手机页/工具报缺 `cv2` | rootfs 没装 `python3-opencv` | §3.4 包列表已含，漏了就补 |

## 9. 迁移后要验证的（"后续路线"其实已经走完了）

现系统（Debian + Foxy）上这些**都已完成**：底盘驱动 ✅、LD14P 雷达 ✅、slam_toolbox 建图与定位 ✅、
自研导航（A\* + 纯跟踪 + 安全层，真机 10 次到达 0.149~0.150 m）✅、手机网页遥控/点选/回原点 ✅
（见 08 方案与《05》实验 6）。所以换系统后的目标是**"一模一样地复现"**，而不是从零开发：

| # | 复现项 | 判据 / 命令 |
|---|---|---|
| 1 | 设备与别名 | `robotcheck` → `✓ 预检查通过`；`ls -l /dev/CAR_BASE /dev/LD14P /dev/i2c-6` |
| 2 | 底盘里程计 | `odomhz` 有稳定频率；推车看 `/odom` 变化 |
| 3 | 雷达 | `scanhz`；`scanview` 看点云正常 |
| 4 | **定位（N1）** | `robotnav` + `mapv`：车标与实际相符、推车跟随不跳变 |
| 5 | **规划判据（N2）** | `navcheck --goal 2.0 1.5` → ✓ 通过 |
| 6 | **导航（N3/N4）** | `nav --listen` + 手机点一下 → 到达误差 ≤0.15 m、路径自检有输出 |
| 7 | 手机网页 | 同一 WiFi 打开 `http://<小车IP>:8080/`：地图/摇杆/保存地图/点选/回原点 |
| 8 | 数据 | `04_map/robot_map.*` 与 `logs/` 已带过来；`navplot` 能出图 |

**换完系统再考虑的新东西**（Humble 上都变简单了）：
nav2（代价地图 + 恢复行为，替代自研那部分）、`robot_localization`（N6-I4 的 EKF 融合）、
行为树多航点巡检、自启动 / 看门狗 / 低电回充（= 原 §9 第 5~7 项）。

## 10. 本次换卡执行清单（双卡方案，按顺序打勾）

**原则**：旧卡（Debian，已经能跑）**一个字都不动**，全程可拔卡回退。
（✅ = 2026-09-22 已完成并实测通过）

### 阶段 0 · 准备（约 30 分钟）

- [x] **备份/克隆现卡**（x86 上整盘 `dd`，或至少打包 `~/RobotCode` + `/etc` + `/lib/modules/5.10.160` + `/lib/firmware`）
- [x] §4 的 `toybrick_drivers.tar.gz` 打好、`~/.bash_aliases` 与 `~/RobotCode` 拷到主机/暂存
- [x] **串口接 U-Boot，记录 `printenv bootargs` / `printenv bootcmd` / `part list mmc 1`** —— 决定新卡要不要 p2（§2.5）
- [x] 定路线：有 USB 读卡器 → **方案 A（板内 debootstrap）**；没有 → 方案 B（x86 + qemu）

### 阶段 1 · 做 rootfs（约 1~2 小时）

- [x] 分区 + 格式化：**照现卡布局**（单分区 p1 占满；实测不需要 p2）
- [x] debootstrap（方案 A）或 ubuntu-base 解包（方案 B）
- [x] §3.4 基础包清单（含 `python3-opencv / numpy`，车端工具要）→ 实测 `cv2/numpy/serial` 均可 import
- [x] §4 迁移：`/lib/modules/`**只放 `5.10.160`** ✅ + `/lib/firmware` 32 M ✅ + `99-robot.rules` ✅ + 网络配置 ✅；⚠ 7 个 toybrick 包**未装**（§2.5.2，暂不影响导航）
- [x] §5.4 规则文件放进 `etc/udev/rules.d/`，**补上 `i2c-6` 那一行** ✅（现存 `99-robot.rules` + `99-robot-i2c.rules`）
- [x] §5.3 `fstab` 用 UUID ✅；`/etc/hostname`、时区、`/etc/resolv.conf`、建用户 `toybrick`（`sudo` + `dialout`）✅

### 阶段 2 · 上板首启（约 30 分钟）

- [x] **原系统正常关机 → 拔旧卡 → 插新卡 → 上电**（不要热插拔）
- [x] 串口看 U-Boot：仍按原厂加载内核/dtb ✅（`uname -a` = `5.10.160`）
- [x] §6 上板核查清单逐条过：`/lib/modules/5.10.160` 非空 ✅、`ip a` wlan0 `192.168.31.73` ✅、`/dev/LD14P → ttyACM0`、`/dev/CAR_BASE → ttyCH341USB0` ✅
- [ ] 开 SSH（`systemctl enable ssh`，远程调试时再开）

### 阶段 3 · ROS2 与工程（约 1~2 小时）⬅ **当前在这里**

- [x] **§7 装 ROS2 Humble** ✅（`/opt/ros/humble` 在位，267 个包；**未装 rplidar**）
- [x] 解包 `~/RobotCode` ✅ → §7.1 的 `grep -rn ros2-foxy` 在工程代码里已为 **0 处** ✅
- [x] `rosdep install` ✅ + `colcon build` ✅（`base_driver` / `ldlidar` / `robot_bringup` / `slam_toolbox` 4 个全过）
- [ ] §9 的 8 项复现验证（`robotcheck` → `odomhz`/`scanhz` → `robotnav`+`mapv` → `navcheck` → `nav --listen` → 手机页）⬅ **下一步**

### 回退方案

任何一步卡住：**拔新卡、插回旧卡**，立刻回到现在的状态。新卡能启动但有小毛病时，**也别动旧卡**，
就在新卡上修——旧卡是唯一的安全网。

### 一句话提醒

最容易翻车的只有两点：**① `/lib/modules` 的版本（本机 = `5.10.160`）**；
**② 新卡分区布局与 U-Boot `bootargs` 里 `root=` 是否对得上**。其余都是"缺包/缺规则"，可当场补。

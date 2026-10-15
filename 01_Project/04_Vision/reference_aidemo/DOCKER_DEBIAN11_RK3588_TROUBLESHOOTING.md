# Toybrick RK3588 Debian 11 安装 Docker 踩坑记录

> 结论先行：**官方 Toybrick 固件的内核不支持 Docker，不推荐在该板子上用 Docker**。
> 如果确实需要容器化，建议改用原生安装或重新编译内核（见文末）。

---

## 一、硬件/系统环境

| 项目 | 信息 |
|------|------|
| 开发板 | Toybrick RK3588 (TB-RK3588SD) |
| SoC 架构 | ARM64 / aarch64 (Cortex-A76 + A55) |
| 系统 | Debian GNU/Linux 11 (bullseye) |
| 内核 | 5.10.160 |
| 磁盘 | 58G eMMC |

---

## 二、踩过的坑（按时间顺序）

### 坑 1：Docker Compose 是 V1，需要 V2 插件

老脚本用 `docker-compose`（连字符，V1），新版 Compose 文件去掉 `version` 字段后 V1 直接报错：

```
Unsupported config option for services: 'ros2'
Unsupported config option for volumes: 'ros2_log'
```

**解决**：安装 Compose V2 插件（`docker compose`，空格形式）。

```bash
# 下载 aarch64 二进制（GitHub Releases）
sudo mkdir -p /usr/libexec/docker/cli-plugins
sudo cp docker-compose-linux-aarch64 /usr/libexec/docker/cli-plugins/docker-compose
sudo chmod +x /usr/libexec/docker/cli-plugins/docker-compose

docker compose version
# Docker Compose version v5.3.1
```

> 下载地址：`https://github.com/docker/compose/releases`（找 `docker-compose-linux-aarch64`）
> 或 `.deb` 包：`https://download.docker.com/linux/debian/dists/bullseye/pool/stable/arm64/`

---

### 坑 2：`buildx` 插件缺失

Compose V2 的 `build` 需要 buildx 插件，否则报警告：

```
WARN[0000] Docker Compose requires buildx plugin to be installed
```

**解决**：同样放 CLI 插件目录。

```bash
sudo mkdir -p /usr/libexec/docker/cli-plugins
sudo cp buildx-v0.36.0.linux-arm64 /usr/libexec/docker/cli-plugins/docker-buildx
sudo chmod +x /usr/libexec/docker/cli-plugins/docker-buildx

docker buildx version
```

> 下载地址：`https://github.com/docker/buildx/releases`（找 `buildx-v*.linux-arm64`）

---

### 坑 3：Docker Hub 拉镜像超时（外网不通）

板子访问 `registry-1.docker.io` 超时：

```
Get "https://registry-1.docker.io/v2/": net/http: request canceled while waiting for connection
```

**尝试过的镜像源（均无效）**：
- `https://docker.1ms.run` → 429 限流
- `https://docker.xuanyuan.me` → 429 限流
- `https://docker.m.daocloud.io` → 403 Forbidden
- `https://dockerproxy.net` → 超时
- `https://docker.nju.edu.cn` → 超时

**最终结论**：板子完全无法访问外网拉镜像，只能 PC 离线 `docker save` → 传板子 `docker load`。

---

### 坑 4：Docker daemon 启动失败（内核缺 iptables nat 模块）

配置 daemon.json 后 Docker 起不来：

```
Process: 58626 ExecStart=/usr/sbin/dockerd ...
Main PID: 58626 (code=exited, status=1/FAILURE)
```

**原因**：内核没编译 `iptables` 的 `nat` 表。由于 compose 用 `network_mode: host`，不需要 NAT，禁用即可：

```json
{
  "iptables": false,
  "bridge": "none"
}
```

> 存储驱动会降级为 `vfs`（内核缺 overlay/aufs），磁盘占用更大但能用。

---

### 坑 5：`docker build` 挂载 `/dev/mqueue` 失败（致命）

即使加了 `--privileged`、`seccomp: ALLOW_ALL`、禁用 iptables，容器仍起不来：

```
OCI runtime create failed: container_linux.go:377:
mounting "mqueue" to rootfs at "/dev/mqueue" caused:
mount through procfd: no such device: unknown
```

**原因**：内核**没有编译 `CONFIG_POSIX_MQUEUE`**，Docker 的 OCI runtime 在任何容器启动时都强制挂载 `/dev/mqueue`，内核不支持就直接炸。`--privileged` 也绕不过去。

**尝试过的绕过手段（全部失败）**：
1. `seccomp-profile: /etc/docker/seccomp.json`（全放行）→ 无效
2. `docker run --privileged` → 无效
3. `runc` wrapper（启动前 sed 删掉 mqueue 挂载）→ 无效（真正的 runc 在 `/usr/sbin/runc`，不是 `/usr/bin/runc`）

---

### 坑 6：Robostack / conda 方案也失败

由于外网不通，conda 装 ROS2 包时 robostack 镜像源 404 或网络不可达：

```
ros-humble-ros-base does not exist (perhaps a typo or a missing channel).
Network is unreachable
```

**结论**：这条路也走不通，最终改用官方预编译二进制包（见下文 ROS2 安装）。

---

## 三、根本原因分析

RK3588 板子要正常跑 Docker，内核必须开启（官方 Toybrick 固件**全部缺失**）：

| 配置项 | 状态 | 影响 |
|--------|------|------|
| `CONFIG_POSIX_MQUEUE` | ❌ 未编译 | 任何容器都起不来（致命） |
| `iptables` nat 表 | ❌ 不支持 | Docker 网络/NAT 失效 |
| `overlayfs` | ❌ 不支持 | 存储驱动降级为 vfs |
| `cgroup` / `namespace` | 部分缺 | 容器隔离不完整 |

**官方 Toybrick Debian 固件默认不支持 Docker。**

---

## 四、推荐的替代方案

### 方案 A：原生安装 ROS2（最终采用）

不折腾 Docker，直接装 ROS2：

1. 从 GitHub 下载官方预编译 arm64 包
   ```
   https://github.com/ros2/ros2/releases/download/release-humble-20260220/ros2-humble-20260220-linux-jammy-arm64.tar.bz2
   ```
2. 传到板子解压即可用（注意：这是 Ubuntu 22.04 编译的，Debian 11 上需测试 glibc 兼容性）

### 方案 B：重新编译内核（彻底解决）

适合一定要用 Docker 的场景：

1. 获取 Rockchip SDK 内核源码
2. 打开以下配置重新编译：
   ```
   CONFIG_POSIX_MQUEUE=y
   CONFIG_OVERLAY_FS=y
   CONFIG_NETFILTER_XT_MATCH_*..
   ```
3. 烧写新内核

参考：
- https://blog.csdn.net/weixin_39825996/article/details/151264130
- https://github.com/ShengTong1/rk3588-docker-kernel-guide

### 方案 C：源码编译 ROS2（备选）

```bash
# apt 装依赖 → vcs 拉源码 → colcon build
# 耗时 2-4 小时（RK3588 8 核），磁盘占用约 10GB
```

---

## 五、卸载 Docker（如果已安装）

```bash
sudo apt-get remove --purge -y docker.io docker-compose python3-docker python3-dockerpty
sudo rm -rf /etc/docker /var/lib/docker /var/run/docker.sock
sudo mv /usr/sbin/runc.orig /usr/sbin/runc 2>/dev/null   # 还原 runc wrapper
sudo rm -f /usr/libexec/docker/cli-plugins/docker-compose \
          /usr/libexec/docker/cli-plugins/docker-buildx
```

---

## 六、关键下载地址汇总

| 用途 | 地址 |
|------|------|
| Compose V2 插件 | `https://github.com/docker/compose/releases` |
| buildx 插件 | `https://github.com/docker/buildx/releases` |
| ROS2 预编译 arm64 | `https://github.com/ros2/ros2/releases` |
| Miniforge (ARM64) | `https://github.com/conda-forge/miniforge/releases` |
| 清华 Debian 源 | `https://mirrors.tuna.tsinghua.edu.cn/debian` |
| 清华 PyPI 源 | `https://mirrors.tuna.tsinghua.edu.cn/pypi/web/simple` |
| ghproxy (GitHub 代理) | `https://ghproxy.com/https://github.com/...` |

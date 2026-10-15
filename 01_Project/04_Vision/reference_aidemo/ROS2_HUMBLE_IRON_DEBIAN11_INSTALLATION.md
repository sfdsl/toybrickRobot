# RK3588 Debian 11 上安装 ROS2 Humble / Iron

## 目标

本文件提供在 Toybrick RK3588（Debian 11, aarch64）上安装 ROS2 的可行方案。由于官方 Toybrick Debian 固件内核对 Docker 支持不完整，所以推荐使用原生安装方式，而不是容器化安装。

> 推荐：先安装 ROS2 Humble。如果你要使用 ROS2 Iron，可参照相同流程，但兼容性可能稍差。

---

## 适用范围

- 设备：Toybrick RK3588 / TB-RK3588SD
- 系统：Debian GNU/Linux 11 (bullseye)
- 架构：ARM64 / aarch64
- 安装方式：原生预编译二进制（tarball）

---

## 重要说明

1. 官方 Toybrick Debian 固件内核不支持 Docker（`CONFIG_POSIX_MQUEUE`、`overlayfs`、`iptables nat` 等缺失），因此建议不要继续依赖 Docker。
2. 这个安装方案不是通过 `apt install ros-humble-*` 安装，而是使用 ROS2 官方预编译 tarball，并在 Debian 11 上加载运行。
3. 如果你的板子无法直接访问 Docker Hub 或 ROS2 官网，可先在 PC 上下载 tarball，然后通过 `scp` 或 U 盘传到板子。

---

## 1. 准备系统环境

```bash
sudo apt update
sudo apt install -y locales curl gnupg2 lsb-release build-essential wget unzip python3 python3-pip python3-argcomplete
```

生成并激活 UTF-8 本地化：

```bash
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

确认系统架构：

```bash
uname -m
# 应该输出 aarch64
```

---

## 2. 安装必要依赖

ROS2 运行和编译工作空间需要一些系统依赖。

```bash
sudo apt install -y \
  libasio-dev libtinyxml2-dev libeigen3-dev libopencv-dev \
  python3-colcon-common-extensions python3-rosdep2 python3-vcstool
```

> 如果 Debian 11 中某些包名不可用，可用 `python3 -m pip install -U colcon-common-extensions argcomplete rosdep` 作为补充。

安装 Python 依赖：

```bash
python3 -m pip install -U colcon-common-extensions argcomplete rosdep
```

---

## 2.1 如果已经安装 Miniforge

如果你的板子已经安装了 Miniforge，比如常见路径 `/home/toybrick/ros2_humble/miniforge`，可以直接使用已有 Conda 环境来安装 ROS2，而无需重新安装 Miniforge。

```bash
# 进入已安装的 Miniforge 目录
cd /home/toybrick/ros2_humble/miniforge

# 激活 conda shell
eval "$(./bin/conda shell.bash hook)"
conda activate base

# 先确保 conda 可以正常工作
conda --version
```

如果你已经有 `.condarc`，请确认它包含国内镜像源，例如清华镜像和 conda-forge / robostack 频道，以便加速安装。

```yaml
channels:
  - https://mirrors.tuna.tsinghua.edu.cn/anaconda/cloud/robostack/
  - https://mirrors.tuna.tsinghua.edu.cn/anaconda/cloud/conda-forge/
  - conda-forge
  - robostack
```

然后创建 ROS2 Humble 环境：

```bash
conda create -y -n ros2_humble \
  -c conda-forge -c robostack \
  ros-humble-ros-base \
  colcon-common-extensions \
  rosdep \
  compilers \
  cmake \
  pkg-config \
  ninja \
  make
```

激活环境：

```bash
conda activate ros2_humble
```

如果你已经在 `.bashrc` 中添加了 Miniforge 初始化，也可以直接重新登录让 `conda` 可用。

---

## 3. 下载 ROS2 预编译包

### Humble

在板子上直接下载：

```bash
cd /tmp
wget https://github.com/ros2/ros2/releases/download/release-humble-20260220/ros2-humble-20260220-linux-jammy-arm64.tar.bz2
```

如果板子无法访问外网，则在 PC 上下载后传输到板子：

```bash
# PC 上下载
wget https://github.com/ros2/ros2/releases/download/release-humble-20260220/ros2-humble-20260220-linux-jammy-arm64.tar.bz2
# 传到板子，例如 scp
scp ros2-humble-20260220-linux-jammy-arm64.tar.bz2 user@rk3588:/tmp/
```

### Iron（可选）

如果你要尝试 ROS2 Iron，可用类似命令下载 Iron tarball并替换下面的路径。

---

## 4. 解压 ROS2

```bash
sudo mkdir -p /opt/ros/humble
sudo tar xf /tmp/ros2-humble-20260220-linux-jammy-arm64.tar.bz2 -C /opt/ros/humble --strip-components=1
```

如果安装 Iron，则将目标路径改为 `/opt/ros/iron`。

---

## 5. 环境配置

将 ROS2 环境加入 shell 启动项：

```bash
echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
source ~/.bashrc
```

立即启用：

```bash
source /opt/ros/humble/setup.bash
```

如果你使用 zsh：

```bash
echo 'source /opt/ros/humble/setup.bash' >> ~/.zshrc
source ~/.zshrc
```

---

## 6. 验证安装

```bash
ros2 --version
ros2 topic list
```

如果 `ros2` 命令正常执行，则说明基本安装成功。

测试运行示例节点：

```bash
ros2 run demo_nodes_py talker
```

如果你不能立即运行 `talker`，先确认 `PYTHONPATH` 是否包含 `/opt/ros/humble/lib/python3.11/site-packages`（或你的 Python 版本对应目录）。

---

## 7. 安装 ROS2 工作空间依赖

如果你要构建 `ros2_ws/src` 下的包，建议先安装工作空间依赖：

```bash
sudo apt install -y python3-colcon-common-extensions python3-rosdep2 python3-vcstool
sudo rosdep init || true
rosdep update
```

然后在工作空间目录中构建：

```bash
cd /userdata/aidemo/ros2_ws
colcon build --packages-select base_driver
source install/setup.bash
```

---

## 8. 可能的兼容性问题及解决方式

### 8.1 如果报错找不到 `libpython3.11.so`

在 Debian 11 上，默认 Python 版本可能是 3.9。请确保已安装对应 Python 3.11 运行时：

```bash
sudo apt install -y python3.11 python3.11-venv python3.11-dev
```

如果系统没有 `python3.11`，可使用 `update-alternatives` 或直接配置 `PYTHON_EXECUTABLE`、`PYTHONPATH`。

### 8.2 如果 `ros2 topic list` 失败

1. 确认已经执行 `source /opt/ros/humble/setup.bash`
2. 检查 `echo $LANG` 是否包含 `UTF-8`
3. 检查 `env | grep PYTHON` 是否出现混合 Python 版本

### 8.3 如果 `ros2` 命令无法找到包

```bash
echo $AMENT_PREFIX_PATH
```

确保该变量包含 `/opt/ros/humble`。

---

## 9. 推荐安装流程总结

1. `sudo apt update`
2. `sudo apt install -y locales curl gnupg2 lsb-release build-essential python3 python3-pip python3-argcomplete`
3. `sudo locale-gen en_US.UTF-8`
4. 下载 ROS2 tarball 到板子或从 PC 传输
5. 解压到 `/opt/ros/humble`
6. `source /opt/ros/humble/setup.bash`
7. 验证 `ros2 --version`

---

## 10. 额外说明

- 如果你要在 Debian 11 上长期使用 ROS2，建议把 `~/.bashrc` 中的 `source /opt/ros/humble/setup.bash` 写入 shell 启动项。
- 如果你要使用 `ros2_ws` 中的代码，构建前一定要安装 `colcon`、`rosdep` 并运行 `rosdep update`。
- 如果你要升级到 ROS2 Iron，可把上述路径和 tarball 名称改为 `iron`。

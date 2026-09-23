#!/bin/bash
# 在 TB-RK3588SD (Ubuntu 22.04.5 arm64, 内核 5.10.160) 上安装 ROS2 Humble
# 用法：sudo bash ~/RobotCode/install_ros_humble.sh     （只需在最开始输一次密码）
set -e
export DEBIAN_FRONTEND=noninteractive

# 只需输一次密码：后台定时刷新 sudo 时间戳，避免长安装中途再要密码
sudo -v
( while true; do sudo -n true; sleep 60; done ) &
KEEP_SUDO=$!
trap 'kill $KEEP_SUDO 2>/dev/null' EXIT

echo "== [1/5] 启用 universe =="
sudo add-apt-repository -y universe >/dev/null 2>&1 || true

echo "== [2/5] 导入 ROS apt key（多源回退，GitHub 慢时自动换） =="
sudo install -d /usr/share/keyrings
for u in \
  https://ghproxy.net/https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  http://repo.ros2.org/repos.key ; do
  echo "  尝试 $u"
  if curl -fsSL -m 60 "$u" -o /tmp/ros.key; then echo "  ✅ 已获取"; break; fi
done
[ -s /tmp/ros.key ] || { echo "[FATAL] ros.key 下载失败"; exit 1; }
sudo gpg --batch --yes --no-tty --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg < /tmp/ros.key

echo "== [3/5] 写 ROS2 源（清华镜像，arm64） =="
echo "deb [arch=arm64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu jammy main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null

sudo apt-get update

echo "== [4/5] 逐包校验（arm64 源里是否都有） =="
PKGS=(
  ros-humble-ros-base
  ros-humble-navigation2 ros-humble-nav2-bringup
  ros-humble-slam-toolbox ros-humble-nav2-map-server          # Humble 里叫 nav2-map-server
  ros-humble-robot-localization
  ros-humble-diff-drive-controller ros-humble-controller-manager
  ros-humble-cv-bridge ros-humble-image-transport
  python3-colcon-common-extensions python3-rosdep
)
MISSING=()
for p in "${PKGS[@]}"; do
  if apt-cache show "$p" >/dev/null 2>&1; then
    printf '  OK   %s\n' "$p"
  else
    printf '  MISS %s\n' "$p"; MISSING+=("$p")
  fi
done
if [ ${#MISSING[@]} -gt 0 ]; then
  echo "[FATAL] 以下包在 arm64 源里不存在：${MISSING[*]}"
  echo "        可改用官方源：http://packages.ros.org/ros2/ubuntu jammy main"
  exit 1
fi

echo "== [5/5] 安装（ROS base + nav2 + slam_toolbox + 工具，不装 rplidar） =="
sudo apt-get install -y "${PKGS[@]}"

echo "== 收尾：rosdep 初始化 =="
sudo rosdep init 2>/dev/null || echo "(rosdep 已初始化，跳过)"
rosdep update || echo "(rosdep update 失败，可稍后重跑)"

echo
echo "✅ 完成。验证："
echo "   source /opt/ros/humble/setup.bash && ros2 --help | head -3"

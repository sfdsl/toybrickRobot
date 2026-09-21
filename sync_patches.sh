#!/usr/bin/env bash
# ============================================================
# 同步 slam_toolbox 的本地适配补丁到 ros2_ws/patches/（备份用）
#
# 背景：ros2_ws/src/slam_toolbox 是上游 clone，**不入 git**（119M）；
#       我们对它的改动（launch / config / CMakeLists）靠 patches/ 里的
#       「diff + 改后文件副本」还原。
#       ⚠ 每次改了 slam_toolbox 里的文件，都要跑一次本脚本，
#         否则补丁会过期（backup 别名已自动先调用它）。
#
# 用法： ~/RobotCode/sync_patches.sh
# ============================================================
set -eu

WS="$HOME/RobotCode/ros2_ws"
SRC="$WS/src/slam_toolbox"
OUT="$WS/patches"

[ -d "$SRC/.git" ] || { echo "✗ 找不到 $SRC（上游 clone 不在？）" >&2; exit 1; }
mkdir -p "$OUT/slam_toolbox_files"

git -C "$SRC" diff > "$OUT/slam_toolbox_local_changes.patch"

n=0
while IFS= read -r f; do
    if [ -n "$f" ]; then
        mkdir -p "$OUT/slam_toolbox_files/$(dirname "$f")"
        cp "$SRC/$f" "$OUT/slam_toolbox_files/$f"
        n=$((n + 1))
    fi
done < <(git -C "$SRC" diff --name-only)

echo "✓ 补丁已同步：$n 个文件（diff $(wc -l < "$OUT/slam_toolbox_local_changes.patch") 行）→ $OUT/"
git -C "$SRC" diff --name-only | sed 's/^/    /'

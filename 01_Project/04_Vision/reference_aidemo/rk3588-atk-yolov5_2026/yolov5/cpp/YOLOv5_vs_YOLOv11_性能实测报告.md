# YOLOv5 vs YOLOv11 在 Toybrick RK3588 上的性能实测报告

> 测试时间：2026-07-29
> 测试板子：Toybrick RK3588（Debian 11，aarch64）
> NPU：RK3588 内置 6 TOPS，3 个独立核
> 运行时：librknnrt 2.3.2（2025-04-09）
> OpenCV：4.5.1（系统原生）
> 工具链：本机原生编译（gcc/g++ aarch64）

---

## 一、测试对象

| 工程 | 路径 | 模型 | 规格 | 输入 | 量化 |
|---|---|---|---|---|---|
| YOLOv5 | `rk3588-atk-yolov5_2026/yolov5/cpp` | yolov5.rknn | yolov5s | 640×640 | INT8 |
| YOLOv11 | `rk3588-atk-yolov11-2026/yolo11/cpp` | yolo11.rknn | yolo11s（DFL 外移版，9 路输出） | 640×640 | INT8 |

两套工程均在板子上**原生编译**（yolo11 的 `CMakeLists.txt` 已加 native 分支对齐 opencv4，`main.cc` 已加 `use_gui` 门控与分段计时），保证同工具链、同 OpenCV、同口径。

---

## 二、测试方法

### 2.1 统一外壳（公平对比）
为了让对比只反映"框架/NPU 效率"而非外壳差异，两套都套用相同优化外壳：
- 采集与推理解耦：独立 cap 线程抓最新帧，推理线程处理最新帧
- 预分配 letterbox 输出缓冲（避免每帧 malloc/free）
- `CAP_PROP_BUFFERSIZE=1` 降低端到端延迟
- 分段计时：`pre`（letterbox）/ `npu`（rknn_run + outputs_get）/ `post`（NMS + 画框）
- 无 DISPLAY 命令行模式运行，每 30 帧打印一次分段耗时与 FPS

### 2.2 三核吞吐测试（YOLO_BENCH）
实时摄像头受 USB 摄像头 ~30fps 上限限制，无法测出 NPU 真实吞吐。故对 yolov5 三核版本额外用 `YOLO_BENCH=1` 模式：抓 1 帧后循环喂同一帧给 worker，绕开摄像头上限，测 NPU 真实吞吐。

---

## 三、结果

### 3.1 YOLOv5s vs YOLOv11s（同等外壳，实时摄像头）

| 指标 | YOLOv5s | YOLOv11s | 说明 |
|---|---|---|---|
| **NPU 推理**（rknn_run+outputs_get） | **~23.2 ms** | **~34 ms** | 干净的框架指标，yolov5 快约 47% |
| 预处理 letterbox | ~4.5 ms | ~24 ms* | 见注 |
| 后处理 NMS+画框 | ~0.8 ms | ~1.4 ms | 接近 |
| **端到端 FPS（无窗口）** | **~32（30–38）** | **~15–20** | 均受摄像头 ~30fps 限制 |
| 检测 | 正常 | 正常 | — |

> \* yolo11 的 `pre=24ms` 偏高是 GStreamer 后端给的源帧分辨率偏大 + RGA letterbox fallback 到 CPU 导致，**非框架本身差异**（两套用同一 `convert_image_with_letterbox`）。真正衡量框架 NPU 效率的是 `npu` 一项。

### 3.2 YOLOv5 三核 NPU 多线程（bench 真实吞吐）

- 单核 NPU：23ms/帧 → 理论上限 1/0.023 ≈ **43 FPS**
- 三核 NPU（3 上下文 + `rknn_set_core_mask` 绑核 0/1/2 + 3 worker 线程）：**960 帧 / 12s = 80 FPS 持续吞吐**
- 加速比：80 / 43 ≈ **1.86x**（近 2x，未达 3x，因 letterbox 的 RGA/CPU 预处理、后处理画框、线程同步有共享开销）

### 3.3 三核对实时摄像头 FPS 的影响

| 场景 | 单核 | 三核 |
|---|---|---|
| bench 纯 NPU 吞吐 | ~43 FPS（理论） | **~80 FPS（实测）** |
| 实时摄像头 | ~32 FPS | ~30 FPS |

**实时摄像头 FPS 单核/三核都是 ~30**——瓶颈是 USB 摄像头 640×480 只输出 ~30fps，NPU 单核（43fps 容量）本就喂得过来，三核的 80fps 容量用不上。

---

## 四、关键结论

1. **框架对比**：在 RK3588 NPU 上，**YOLOv5s 比 YOLOv11s 更高效**（NPU 23ms vs 34ms），且差距是框架本身带来的，不是外壳造成的。
   - yolov5 在 RKNN 上打磨成熟、3 分支输出干净；
   - yolo11 是 DFL 外移版，9 路输出让 `rknn_outputs_get` 和 NPU 图开销更高，部分算子在 RK3588 NPU 上优化不足。
   - "架构更新"在 GPU/CPU 基准里更优，但在这颗 NPU 上不成立。

2. **三核 NPU**：真有 ~2x 吞吐（80 FPS），**但当前摄像头被 30fps 卡死，实时用不上**。

3. **librga "Failed" 不是瓶颈**：letterbox 的 resize 早就在 RGA 上跑（无 `fallback to CPU` 日志），只有 imfill（背景填色）失败并有 `memset` 快速回退。`pre` 已只有 4.5ms，修掉 imfill 几乎不提 FPS（~0.1）。真正瓶颈是 NPU 23ms。

4. **实时 FPS 的真正瓶颈是摄像头（~30fps），不是 NPU**。

---

## 五、建议

### 想提实时 FPS
- **换高帧率摄像头**：640×480 能输出 60/90fps 的 USB 摄像头或 MIPI 摄像头。此时单核 NPU（43fps）成为新瓶颈，**三核（80fps）可把实时推到 ~60–80 FPS**，三核才有用武之地。
- **不换摄像头**：三核对实时无增益；只能降单帧 NPU 耗时（更小模型 / 更低分辨率，如 480×480）。

### 想要精度且不在乎 FPS 降一半
- 可换 yolo11，但接受 NPU 多 ~11ms/帧，且必须用 rknn_model_zoo 官方脚本正确导出（DFL 外移、加置信度求和分支），否则会比 v5 更慢。

### 离线/视频流处理（无摄像头 30fps 上限）
- 三核 ~80 vs 单核 ~43，近 2x 加速，这时三核值得。

---

## 六、复现命令

### 6.1 yolov5 单核（实时，无窗口）
```bash
cd rk3588-atk-yolov5_2026/yolov5/cpp/install/rk3588_linux_aarch64/rknn_yolov5_cam
export LD_LIBRARY_PATH="$(pwd)/lib:$LD_LIBRARY_PATH"
env -u DISPLAY ./rknn_yolov5_cam ./model/yolov5.rknn 0
# 输出形如: FPS: 32.4 | pre=4.3ms npu=23.5ms post=0.8ms
```

### 6.2 yolo11 单核（实时，无窗口）
```bash
cd rk3588-atk-yolov11-2026/yolo11/cpp/build
export LD_LIBRARY_PATH=../install/rk3588_linux_aarch64/atk_rknn_yolo11_cam/lib:$LD_LIBRARY_PATH
env -u DISPLAY ./rknn_yolo11_cam ../install/rk3588_linux_aarch64/atk_rknn_yolo11_cam/model/yolo11.rknn 0
# 输出形如: YOLO11 FPS: 15.x | pre=.. npu=34.x post=..
```

### 6.3 yolov5 三核 NPU（bench 真实吞吐）
```bash
cd rk3588-atk-yolov5_2026/yolov5/cpp/install/rk3588_linux_aarch64/rknn_yolov5_cam
export LD_LIBRARY_PATH="$(pwd)/lib:$LD_LIBRARY_PATH"
env -u DISPLAY YOLO_BENCH=1 ./rknn_yolov5_cam ./model/yolov5.rknn 0
# 看末行 processed=N，N/12 即为吞吐 FPS（实测 ~80）
```

### 6.4 编译
```bash
# yolov5（已含 build_native.sh）
cd rk3588-atk-yolov5_2026/yolov5/cpp && bash build_native.sh

# yolo11（CMakeLists 已加 native 分支）
cd rk3588-atk-yolov11-2026/yolo11/cpp
rm -rf build && mkdir build && cd build
cmake .. -DTARGET_SOC=rk3588 -DCMAKE_BUILD_TYPE=Release && make -j8
```

---

## 七、代码改动记录

为本次测试对两套工程做的改动：

### yolov5（`rk3588-atk-yolov5_2026/yolov5/cpp`）
- `main.cc`：采集/推理解耦 + 预分配 + 分段计时 + RGB→BGR + 全屏 + 触摸红 X 关闭（这些是性能测试之外的功能改进）。
- `main.cc`：新增三核 NPU 多线程版本（3 上下文 + `rknn_set_core_mask` 绑核 0/1/2 + 3 worker 线程 + 有界帧队列/结果集合 + `YOLO_BENCH` 模式）。
- `CMakeLists.txt`：native 分支对齐 opencv4、移除 GTK 依赖、传 `TARGET_SOC=rk3588`（启用 `LIBRGA_IM2D_HANDLE`）。

### yolo11（`rk3588-atk-yolov11-2026/yolo11/cpp`）
- `CMakeLists.txt`：加 ATK_TOOLCHAIN 探测 + native 分支（pkg-config opencv4），原本只支持交叉编译。
- `main.cc`：套用与 yolov5 相同的优化外壳（采集解耦 + 预分配 + 分段计时 + RGB→BGR + `use_gui` 门控），用于公平对比。

> 注：yolov5 的 `main.cc` 当前为三核多线程版本；若要回到单核版本，参见 git 历史或注释中的单核结构。

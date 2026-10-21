# MQTT 功能测试说明（P3 · 卡-04/卡-10）

更新：2026-10-21。适用：Toybrick RK3588 + ROS 2 Humble + 云端 EMQX Broker。
对应验收：《04_执行检验/卡-10-语音与远程入口.md》放行条件 #2/#3/#4（#1 断网停止见 §5，#5 ESP32 另卡）。

---

## 0. 测试对象与原理（先看懂再动手）

MQTT 功能 = **小车上的 `mqtt_bridge.py`** 与云端 Broker 之间的双向桥：

```text
云端侧（你测试用的替身）                小车侧（必须跑代码！）
EMQX 面板 WebSocket 客户端   ←──MQTT──   mqtt_bridge.py ──localhost HTTP──→ robot_ui.app
（订阅 robot/car01/# 看遥测）             （TLS/租约/幂等/seq）   （统一任务入口/意图执行）
（发布 robot/car01/task 下任务）                    │
                                        （可选）bringup + nav2 + SLAM → 真实位姿/电量/走目标
```

**关键认知：面板 WebSocket 客户端只是「云端那一端的替身」**（代替外网监控/遥控器）。
`robot/car01/#` 的发布者是小车上的 `mqtt_bridge.py`——**小车侧不跑桥，订阅再久也收不到任何消息**。

---

## 1. 测试环境与准备

| 项 | 值 |
|---|---|
| 云端 Broker | `42.192.211.136`（EMQX） |
| 面板（管理/WS 测试入口） | `http://42.192.211.136:18083` |
| MQTT 明文端口 | `1883`（⚠ TLS 端口 8883/8084 当前不可用，见 §7） |
| WebSocket 端口 | `8083`（Path=`/mqtt`） |
| 测试账号 | `robot_car`；密码：板端 `~/.mqtt_pass` 内容（**勿写回文档/脚本**） |
| 设备 ID | `car01`（桥默认值；话题前缀 `robot/`） |
| 小车网络 | 板子能连外网（ping 42.192.211.136 通） |

**凭证口径（卡-04）**：密码只放 `~/.mqtt_pass`（0600）；文档、命令行、脚本里不写明文。
robot_ui 的鉴权 token 由 app.py 启动时落盘 `~/.robot_ui_token`（0600），桥自动读取。

---

## 2. 消息契约速查（测试要构造/核对的内容）

### 上行（小车 → 云端，订阅 `robot/car01/#` 观察）

| 主题 | 频率 | 内容 |
|---|---|---|
| `robot/car01/presence` | 事件 | `online` / `offline`（retain + 遗嘱：桥异常掉线由 broker 代发 `offline`） |
| `robot/car01/telemetry` | 1 Hz | `{"dev","ts","seq","pose":{x,y,yaw},"vel":{vx,wz},"battery":{voltage,pct},"mode","goal","map_id"}` |
| `robot/car01/result` | 每任务 | `{"req_id","ok","msg"}`（幂等命中时另带 `"duplicate":true`） |

### 下行（云端 → 小车，发布到 `robot/car01/task`，QoS 1）

```json
{"dev":"cloud","ts":1792500000.5,"seq":1,"req_id":"t1","action":"stop","args":{}}
```

| 字段 | 必填 | 说明 |
|---|---|---|
| `dev` | ✓ | 发送方标识（如 `cloud`、`pad1`）；seq 按 dev 独立去旧 |
| `ts` | ✓ | epoch 秒（时间戳乱序防护） |
| `seq` | ✓ | **单调整数**，同 dev 内递增；回退/重复会被桥直接丢弃 |
| `req_id` | ✓ | ≤80 字符唯一 ID；同 ID 同参幂等、同 ID 异参拒绝（300s 窗口） |
| `action` | ✓ | `stop` / `goal` / `intent` / `manual` / `heartbeat` |
| `args` | — | 见下表 |
| `on_disconnect` | — | `continue`（默认）/ `stop`：断链时是否终止该远程 goal |

**action 与 args 对照**：

| action | args 示例 | 行为 |
|---|---|---|
| `stop` | `{}` | 全停：取消巡逻/探索/导航/手动（最高优先） |
| `goal` | `{"name":"充电位"}` | 查 `robot_map.places.json`，命中才导航（换图/没这名字拒绝） |
| `goal` | `{"x":6.61,"y":-1.02}` | 直接发坐标目标（可带 `map_id` 校验） |
| `intent` | `{"intent":"forward","args":{"duration":1.5}}` | 结构化意图（同语音词表：stop/forward/back/left/right/home/goto_place/patrol_start/patrol_stop） |
| `manual` | `{"vx":0.10,"wz":0.0,"lease_ms":600}` | 手动速度租约：期内桥按 10Hz 续发，心跳断 → 600ms 内补零停车 |
| `heartbeat` | `{}` | 仅给 manual 租约续期，不改速度 |

**安全约束**（测试中应观察到）：manual 的 vx 钳位 ±0.20；intent 的 forward/back 限速 ≤0.10 m/s、限时自动归零；任何命令不绕过 collision_monitor。

---

## 3. 第 1 层：离线自测（30 秒，不需要网络/真车）

```bash
cd ~/RobotCode/01_Project/03_SLAM
python3 mqtt_bridge.py --selftest      # 桥契约：payload 校验 / seq 去旧 / 心跳租约
python3 robot_ui/_selftest_intent.py   # 意图层：幂等/去抖/地点表/限速限时
python3 robot_ui/serial_cmd.py --demo  # 串口协议：ACK/非法帧/stop 直通
```

三个都打印 `ALL PASS` → 代码逻辑层通过，进第 2 层。

---

## 4. 第 2 层：云端联调（不动车，验证全链路消息）

### 4.1 小车侧：跑「最简两样」（不需要 bringup/底盘）

```bash
# 终端 1：统一任务入口（桥的下行任务要经它转发）
source /opt/ros/humble/setup.bash && source ~/RobotCode/ros2_ws/install/setup.bash
cd ~/RobotCode/01_Project/03_SLAM && python3 -m robot_ui.app --no-auth

# 终端 2：MQTT 桥（小车侧的本体）
source /opt/ros/humble/setup.bash && source ~/RobotCode/ros2_ws/install/setup.bash
cd ~/RobotCode/01_Project/03_SLAM
python3 mqtt_bridge.py --host 42.192.211.136 --port 1883 --no-tls \
    --user robot_car --pass-file ~/.mqtt_pass
```

看板判断桥起来了没：终端 2 出现 `MQTT 已连接 42.192.211.136:1883 ｜ 订阅 robot/car01/task`。
（日志：`/tmp/mqtt_bridge.log`、`/tmp/robot_ui.log`，若用 start_robot.sh 启动则见 §5。）

> **没底盘/雷达/SLAM 时的正常现象**：telemetry 里 `pose/vel/battery` 为 `null`；
> `goal` 任务回 `ok:false`（定位 TF 缺失）。这只说明链路通，不是功能失效。

### 4.2 云端侧：EMQX 面板 WebSocket 客户端

1. 浏览器打开 `http://42.192.211.136:18083`，登录面板（管理员账号）。
2. 左侧菜单 **工具 → WebSocket**（或「WebSocket 客户端」），新建连接：
   - Host `42.192.211.136`，Port `8083`，Path `/mqtt`，SSL 关
   - Username `robot_car`，Password `~/.mqtt_pass` 的内容
   - 点 **Connect** → 状态 Connected
3. **订阅**：Topic 填 `robot/car01/#`，QoS 1，Subscribe。
   → 应立即看到 `presence: online`（retain），随后每秒一条 `telemetry`。
4. **发布**：Topic 填 `robot/car01/task`，QoS 1，把 §2 的 JSON 贴进消息框，Publish。

### 4.3 测试用例（按顺序做；重发同 req_id 时 seq 必须递增！）

| # | 目的 | 发布内容（`robot/car01/task`） | 期望（`robot/car01/result` / 现象） | 对应 |
|---|---|---|---|---|
| T1 | 停止链路 | `{"dev":"cloud","ts":...,"seq":1,"req_id":"t1","action":"stop","args":{}}` | 回执 `ok:true, msg:"已停止…"` | 卡-10 #1 |
| T2 | 去地点全链 | `{"dev":"cloud","ts":...,"seq":2,"req_id":"t2","action":"goal","args":{"name":"充电位"}}` | 有 TF：出发；无 TF：`ok:false` 提示定位未更新（=链路通） | #2 |
| T3 | 地点不存在 | `{"dev":"cloud","ts":...,"seq":3,"req_id":"t3","action":"goal","args":{"name":"不存在的地方"}}` | `ok:false`，msg 含「不存在」，车不动 | 卡-05 |
| T4 | 手动+租约 | `{"dev":"cloud","ts":...,"seq":4,"req_id":"t4","action":"manual","args":{"vx":0.10,"wz":0.0,"lease_ms":600}}` | 有底盘：缓行 0.6s 自动停；并见回执「心跳超时，已停车」 | #3 |
| T5 | 结构化意图 | `{"dev":"cloud","ts":...,"seq":5,"req_id":"t5","action":"intent","args":{"intent":"forward","args":{"duration":1.5}}}` | `ok:true`；有底盘则前进 1.5s 自动归零 | #2 |
| T6 | **幂等** | T1 再发一遍，**req_id 仍为 `t1`，seq 用新的**（如 6） | 回执 `duplicate:true`，**不重复执行** | #2 |
| T7 | **乱序丢弃** | 发 `seq` 回退的（如 `seq:3`，比刚才小） | 桥静默丢弃，**无新回执** | #4 |
| T8 | 词表外拒绝 | `{"dev":"cloud","ts":...,"seq":7,"req_id":"t8","action":"intent","args":{"intent":"fly"}}` | `ok:false` 词表拒绝 | 卡-05 |
| T9 | 超长/坏帧 | 发一串 5000 字符，或 `not json` | `ok:false` 拒绝，桥不崩、车不停 | 卡-04 |

> ⚠ **T6 的坑**：重发同 `req_id` 时如果 `seq` 也照抄，会先被乱序防护丢掉（表现为"没反应"），
> 测幂等必须**同 req_id、递增 seq**；测乱序才用回退 seq。

### 4.4 鉴权与断链（卡-04 #4 另两项）

**T10 鉴权**：面板 WebSocket 用**错误密码**连 `8083` → 连接被拒（`Bad user name or password`，rc=134）。
（无账号连接同理被拒。）

**T11 断链遗嘱 + 重连恢复**：
1. 订阅着 `robot/car01/#`，在小车上 `pkill -9 -f mqtt_bridge.py`（无 DISCONNECT → 触发遗嘱）
2. → 数秒内收到 `presence: offline`（broker 代发）
3. 重启桥（§4.1 终端 2 命令）→ 收到 `presence: online` **再次**出现
4. 立刻发一条 `{"dev":"cloud","ts":...,"seq":1,"req_id":"t11","action":"stop","args":{}}`
   → 有回执 = **重连后恢复订阅成功**（红队坑「重连不重订阅」的验证点）

---

## 5. 第 3 层：实车测试（第 2 层全过后）

### 5.1 启动

```bash
# ⚠ 先把车架空或留出安全距离；手机语音屏红色「停 止」、遥控屏「停止」随时待命
robot --nav2 --mqtt --mqtt-args "--host 42.192.211.136 --port 1883 --no-tls --user robot_car --pass-file ~/.mqtt_pass"
```
一条命令带上底盘/雷达/SLAM/nav2/网页 UI/MQTT 桥。telemetry 里此时有真实 `pose/vel/battery`。

### 5.2 实车用例

| # | 目的 | 操作 | 期望 | 对应 |
|---|---|---|---|---|
| R1 | **远程租约停车** | 发 T4 后**停发一切消息** | 车 600ms 内停住；再按 200ms 间隔连发 `manual`/`heartbeat` 车持续动，**一断就停** | #3 |
| R2 | **断网本地停止** | 拔网线/关 WiFi（云端失联）后，按语音屏红钮 / 遥控「停止」 | 车**照常停住**（本地链路不依赖云） | #1 |
| R3 | 同源不自锁 | 先发 T4（manual），3 秒内发 T2（goal） | goal 正常执行，不报「被 TELEOP 占用」 | 已修缺陷回归 |
| R4 | goal 实走 | 面板发 `{"name":"充电位"}`（地点表里已存） | 车开到充电位；result 回执 ok | #2 |
| R5 | 真实障碍兜底 | goal 途中人从车前走过 | 绕行/减速/急停（雷达+collision_monitor），**虚拟墙/编辑不关安全层** | 路线图 P2/P3 口径 |

---

## 6. 验收对照（做完后在卡-10 打勾）

| 卡-10 放行条件 | 用例 | 现状（2026-10-20 云端实测） |
|---|---|---|
| #2 重复命令不重复执行 | T6 | ✓ 已验（duplicate 透传实测） |
| #3 远程掉线按租约停车 | T4 / R1 | ✓ 云端侧已验（0.7s 补零）；实车 R1 待测 |
| #4 重连/鉴权/乱序 | T7/T10/T11 | ✓ 已验（遗嘱 4/4、错密码 rc=134、乱序丢弃） |
| #1 断网本地停止 | R2 | 待实车 |
| #5 ESP32 串口协议 | — | 协议与桩就绪，等固件卡 |

---

## 7. 常见问题排查

| 现象 | 原因 | 处理 |
|---|---|---|
| 面板订阅 `robot/car01/#` 一直空白 | **小车侧桥没跑** / 设备 ID 不对 | 小车跑 §4.1 终端 2；话题按 `robot/car01/#` 核对 |
| 桥起不来：`--host 必填` | 没给 `--host` | 按 §4.1 命令补全 |
| 桥日志：`本地 HTTP 失败 … Connection refused` | `robot_ui.app` 没起 | 先起终端 1 再起终端 2 |
| result：`定位 map→odom 超过 2 秒未更新` | 无 SLAM/TF（联调阶段正常） | 上实车/起 bringup 后才有真实 goal |
| T6 看不到 `duplicate` | seq 照抄被乱序防护先丢了 | 同 req_id + **递增 seq** |
| 发 manual 车不动（实车） | 租约 600ms 到期即停；或 vx 超 0.20 被钳位 | 连发续租（≤10Hz）；vx 用 0.10 |
| 连 8883/8084 超时或 SSL 错 | **云端 TLS 监听当前未修好** | 暂用 1883 `--no-tls`；修好后改 `--tls-ca`（卡-10 遗留项） |
| 启动报端口占用 | 上次进程残留 | `robotstop` 或 `pkill -9 -f robot_ui.app; pkill -9 -f mqtt_bridge.py` |

## 8. 安全须知

1. 实车测试**先测 T1 stop**，确认刹车链路再测会动车的用例；车轮架空最稳。
2. 任何语音/MQTT/串口命令都不绕过 `control_mux → collision_monitor` 安全链；
   硬件急停（断电）始终是最后一级，**不依赖网络**。
3. 密码只存 `~/.mqtt_pass`（0600）；测试中**不要把密码贴进文档、脚本、命令行历史**。
4. 公网 Broker：用完可在面板断开测试连接；安全组只放行 1883/8083/18083（用哪个放哪个）。
5. `--no-tls` 仅限联调。生产必须等 8883 TLS 修好后切加密（卡-04 红线：不得明文传输）。

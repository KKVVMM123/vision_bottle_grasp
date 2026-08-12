单右臂 + 右手灵巧手 + 头部 RealSense，GUI 画面点击目标 → 机器人抓取 → 抬升回退保持。
**不含底盘、下单、语音**。

## 运行流程

```
[Python 视觉服务]                          [C++ 主控 bottle_grasp]
RealSense → YOLO-Seg → 掩码 → 深度点云重心
  → 头部角 + 标定 → 机器人 base 系坐标       ← CLIENT_READY(头部角)
  → OpenCV 窗口叠加检测框
  → 鼠标点击目标
  → GRASP {class,x,y,z,yaw} ──TCP:12345──▶ 执行:
                                           start_pos → 张开手 → MoveL staging
                                           → MoveL 接近 → 闭合 → MoveL 抬升
                                           → MoveL 回退保持
  ◀─ GET_HEAD_ANGLES（按需）────────────── 实读头部电机角
  ◀─ GRASP_DONE ───────────────────────── 回复结果
```

## 目录结构

```
bottle_grasp/
├── src/            # C++ 源码（裁剪自商超 + 新增 motion/vision_client/main）
├── include/        # C++ 头文件（kinematics/function/Ti5_socketcan/aoyi_hand/...）
├── eigen3/         # Eigen 头文件（随商超工程携带）
├── python/         # 视觉服务（vision_server.py / head_camera_to_base.py）
│   └── config/vision.yaml
├── config/motion.txt   # C++ 运动与 CAN 配置
├── calibration/    # 相机→头部末端外参（默认复用商超标定结果）
├── tools/aoyi_can_scan/  # 灵巧手 CAN 扫描工具
├── scripts/        # build.sh / start.sh
└── CMakeLists.txt
```

## 硬件与 CAN 约定

| 设备 | 说明 |
|------|------|
| 右臂 | 7 关节，电机 ID `16..22`，`Robot(-1)` 运动学 |
| 右手灵巧手 | **USB 串口** `/dev/ttyCH343USB1` / HandID `60`（本机 T170C 手走串口，非 CAN；可改 `config/motion.txt`） |
| 头部 | 电机 ID `30/31/32`（roll/pitch/yaw），`hand_socketid_bind()` 自动探测通道 |
| 相机 | 头部 RealSense，彩色 1280x720 + 对齐深度 |
| 标定 | `calibration/camera_to_head_connector_result.yaml` + 连杆 162mm，FLU/RzRyRx 约定 |

## 在 AGX 上构建

```bash
# 1. 部署
rsync -av /home/a/商超程序/bottle_grasp/ ti5robot@192.168.110.121:~/bottle_grasp/

# 2. 机器人端构建（libserial/nlohmann/cmake 已在 AGX）
ssh ti5robot@192.168.110.121
cd ~/bottle_grasp && ./scripts/build.sh
```

## 在 AGX 上运行

```bash
# 终端1：视觉服务（需要桌面会话，NoMachine/本地显示器）
cd ~/bottle_grasp/python
python3 vision_server.py

# 终端2：C++ 主控（需要 root 操作 CAN）
cd ~/bottle_grasp
sudo ./build/bottle_grasp
```

也可以直接用 `./scripts/start.sh`（前台/后台模式均可，使用系统 python3）。

GUI 操作：点击画面中的瓶子 → 机器人抓取 → 抬升回退保持。
快捷键：GUI 窗口 `q` 退出、`r` 重新同步头部角、`h` **回预备位**；
C++ 终端同样支持 `h`（回预备位）、`q`（退出）。

### 安全：dry-run
先把 `config/motion.txt` 里 `dry_run=1`，启动后点击目标只会打印规划，不会动作。确认轨迹合理后再改回 `0`。

## 视觉环境（AGX）

视觉服务**直接使用 AGX 系统 python3（/usr/bin/python3, 3.10）**，不依赖任何
anaconda/conda 虚拟环境。已按下面配方装齐：`torch 2.8.0 / torchvision 0.23.0 /
ultralytics 8.3.241 / opencv 4.12.0 / pyrealsense2 / numpy 1.24.4 / PyYAML`
（`pip install --user` 装入 `~/.local`，未动系统目录）。

快速检查环境：
```bash
python3 -c "import numpy, cv2, yaml, torch, torchvision, ultralytics, pyrealsense2; print('视觉环境 OK')"
```

缺失时按 安装.txt 的 Jetson 配方安装（Jetson 专用源）：
```bash
python3 -m pip install torch==2.8.0 torchvision==0.23.0 --index-url https://pypi.jetson-ai-lab.io/jp6/cu126
python3 -m pip install "numpy<2" opencv-python PyYAML pyrealsense2 ultralytics
```

> 说明：不要使用 anaconda 的 `chat_env`/`human_interaction_env` 等虚拟环境运行本工程；
> 也不要往 anaconda 里装依赖。统一走系统 python3。

模型默认 `~/models/yolo11n-seg.pt`（COCO bottle/cup），可改 `python/config/vision.yaml`。

## 测试

### 本机（无需机器人/无需 torch）
```bash
cd python && python3 test_geometry.py          # 几何数学单测
python3 vision_server.py --offline RGB.png DEPTH.png --mock   # 离线全链路（模拟掩码）
```

### 机器人
1. **灵巧手扫描**：`cd tools/aoyi_can_scan && make && sudo ./aoyi_can_scan scan`，确认右手 `can7/ID60`，如不同改 `config/motion.txt`。
2. **头部角核对**：启动 C++ 后看 `CLIENT_READY` 打印的 roll/pitch/yaw 是否与实物一致。
3. **视觉 GUI**：点击瓶子打印机器人系坐标，与人工测量对比（<2cm 再继续）。
4. **dry-run**：`dry_run=1` 跑完整规划。
5. **真机低速抓取**：桌面放一个瓶子，点击 → 抓取 → 抬升回退保持；重复多次记录成功率。

## 关键约定与注意事项

- 视觉给的是瓶子**重心**；手在 TCP 前方 0.15m，因此抓取点必须加**深度补偿**：`grasp_depth_offset_m=-0.03`（停在瓶身前表面，避免手指撞瓶；按瓶径调 0.025~0.045）。
- staging 在目标前方 `approach_x_offset_m`（0.10m），抓后抬升 `lift_height_m`（0.03m）+ 回退 `retreat`（默认 -0.03m x）。
- 手姿态可配置：`hand_open_values`（张开/预抓，默认 `0,0,0,0,0,65000`）、`hand_close_values`（闭合，默认 `30000,...,65000`）。
- 目标需在右臂工作空间内（距离 <0.7m）；瓶子放机器人前方约 0.5m、桌面高度（z≈-0.4）。
- **严禁**同时运行另一个会向机械臂/右手 CAN 总线发控制帧的程序（如商超 `move`）。
- 标定矩阵方向为「相机→头部末端」，实机须做多头部姿态固定瓶验证；未通过前只用 dry-run。
- 头部角由 C++ 每次实读电机角度上报，操作员移动头部后 GUI 按 `r` 或等待 TTL 过期自动重新同步。

---

## TCP 协议（C++ ↔ Python，端口 12345）

Python 为 TCP **服务端**，C++ 为**客户端**，每行一条 JSON，以 `\n` 结尾。

| 方向 | 消息 | 说明 |
|------|------|------|
| C++ → Python | `{"type":"CLIENT_READY","head_roll":..,"head_pitch":..,"head_yaw":..}` | 连接后 / 被询问时上报头部角（度） |
| Python → C++ | `{"type":"GET_HEAD_ANGLES"}` | 询问当前头部角 |
| Python → C++ | `{"type":"GRASP","class_name":"bottle","x":..,"y":..,"z":..,"yaw_deg":..}` | 下发抓取目标（机器人 base 系，米） |
| C++ → Python | `{"type":"GRASP_DONE","result":"ok\|not_grasped\|motion_error","holding":true,"ret":0}` | 抓取结果；`holding` 表示是否仍在保持 |
| Python → C++ | `{"type":"RELEASE"}` | 要求张开手并回预备位 |
| C++ → Python | `{"type":"RELEASE_DONE","result":"ok"}` | 释放完成 |
| Python → C++ | `{"type":"STATUS"}` | 查询状态 |
| C++ → Python | `{"type":"STATUS","holding":..,"head_can_id":..}` | 状态回复 |
| Python → C++ | `{"type":"QUIT"}` | 退出 C++ 主控 |

抓取执行序列（C++ 内部）：`MoveJ 预备位 → 张开手 → MoveL staging → MoveL 接近 → 闭合 → 校验手状态 → MoveL 抬升 → MoveL 回退保持`。若上一轮仍在保持，收到新 `GRASP` 时会先自动释放再抓取。

---

## 配置参数参考

### `config/motion.txt`（C++）

| 键 | 默认 | 说明 |
|----|------|------|
| `tcp_host` / `tcp_port` | `127.0.0.1` / `12345` | 视觉服务地址 |
| `right_hand_can_channel` / `right_hand_can_id` | `7` / `60` | 右手灵巧手 CAN 通道与 HandID |
| `left_hand_can_channel` / `left_hand_can_id` | `6` / `70` | 左手（本工程只用右手，保留作状态查询用） |
| `approach_x_offset_m` | `0.10` | staging 点相对目标向机器人侧偏移（米） |
| `grasp_yaw_deg` | `0.0` | 抓取腕部偏航角（度） |
| `lift_height_m` | `0.03` | 抓取后抬升高度（米） |
| `retreat_x_m` / `retreat_y_m` / `retreat_z_m` | `-0.03 / 0 / 0` | 回退保持点相对抬升点的偏移（米） |
| `move_velocity_mps` | `0.20` | 常规 MoveL 速度 |
| `approach_velocity_mps` | `0.10` | 接近目标速度 |
| `dry_run` | `0` | `1`=只打印规划不动作（安全验证） |

### `python/config/vision.yaml`（Python）

| 键 | 默认 | 说明 |
|----|------|------|
| `camera.serial` | `""` | RealSense 序列号，留空自动选择 |
| `camera.width/height/fps` | `1280/720/15` | 彩色与深度分辨率/帧率 |
| `camera.depth_scale` | `0.001` | 深度每单位对应米数（16bit mm） |
| `camera.min/max_depth_m` | `0.20 / 2.50` | 有效深度范围 |
| `model.path` | `~/models/yolo11n-seg.pt` | YOLO 分割模型路径 |
| `model.device` | `"0"` | CUDA 设备号；CPU 用 `"cpu"` |
| `model.confidence` | `0.35` | 检测置信度阈值 |
| `model.classes` | `[bottle, cup]` | 参与抓取的类别名，空=全部 |
| `calibration.yaml` | `../calibration/...yaml` | 相机→头部末端外参（相对 `python/`） |
| `calibration.link_length_mm` | `162.0` | 头部末端在 base 系 z 向高度 |
| `head.default_*_deg` | `0/30/0` | C++ 未连接时使用的头部角 |
| `head.angles_ttl_sec` | `1.0` | 头部角缓存有效期 |
| `tcp.host/port` | `127.0.0.1 / 12345` | TCP 服务监听地址 |
| `grasp.yaw_deg` | `0.0` | 下发给 C++ 的抓取偏航角 |

---

## AGX 环境安装清单

### 系统与 CAN
- CAN 驱动：若使用商超系统盘一般已带；否则装 `kcan_5.15.148_arm64_rt.deb`（在商超工程目录），确认 `ip link` 能看到 `can0~can7`。
- 编译工具：`sudo apt install -y cmake g++ pkg-config libusb-1.0-dev libserial-dev nlohmann-json3-dev`。
- CAN 波特率：程序启动时自动执行 `sudo ip link set canX ... bitrate 1000000`，**必须 root 运行**。

### Python 视觉环境（Jetson 配方，按 `安装.txt`）
```bash
python3 -m pip install torch==2.8.0 torchvision==0.23.0 --index-url https://pypi.jetson-ai-lab.io/jp6/cu126
python3 -m pip install "numpy<2" opencv-python PyYAML pyrealsense2 ultralytics
```
> 注意：`numpy` 必须 <2（与 ultralytics/torch 兼容）；不要用 anaconda 的 python3.13 环境。

### 显示
- GUI 需要桌面会话：NoMachine 连接 AGX，或在 AGX 本地显示器上运行；纯 ssh 终端无法显示窗口。

---

## 现场调试与标定流程

1. **手/头确认**：`sudo ./tools/aoyi_can_scan/aoyi_can_scan scan` 找到右手通道/ID；启动 C++ 后核对 `CLIENT_READY` 的头部角与实物一致。
2. **坐标对比**：视觉窗口点击瓶子，打印机器人系坐标；用卷尺/激光测距对比 x/y/z，误差 <2cm 再进行下一步。
3. **多姿态标定验证**：同一固定瓶子，在 ≥2 个头部俯仰/横滚姿态下采样，目标在 base 系应基本不变；未通过前**只用 dry-run**。
4. **dry-run**：`dry_run=1`，点击目标只打印 staging/grasp/lift/retreat 四点，确认轨迹合理、无 IK 失败。
5. **低速真抓**：`dry_run=0`，`approach_velocity_mps` 先调小（如 0.05），桌面放瓶子重复抓取，记录成功率与失败原因。

---

## 常见问题排查

| 现象 | 检查方向 |
|------|----------|
| CAN 初始化失败 | 是否 `sudo` 运行；`ip link` 是否有 can0~can7；波特率 1Mbps |
| 灵巧手不动 | `aoyi_can_scan scan` 扫描；确认 `motion.txt` 通道/ID 与实物一致；手是否上电 |
| 手状态查询失败 | 手 ID 与 `right_hand_can_id` 是否一致；查询命令 0x5F 是否被手支持 |
| 视觉服务启动即报 torch/ultralytics 缺失 | 系统 python3 缺 torch/ultralytics，按「视觉环境（AGX）」章节的 Jetson 配方补装；不要用 conda 虚拟环境 |
| 误用了 conda 环境导致 CUDA 库报错（如 libcusparseLt.so.0） | 退出 conda（`conda deactivate`），确认 `which python3` 是 /usr/bin/python3 再运行 |
| 视觉启动报 `Device or resource busy`（相机被占） | 有其他程序在用相机（如 ROS2 `realsense2_camera_node`/`ti5_bottle_grasp bringup`），先停掉它们再启动；可 `fuser /dev/video0` 查占用进程 |
| C++ 一直"等待连接" | Python 服务是否先启动；端口 12345 是否被占用；`tcp_host` 是否 127.0.0.1 |
| 视觉无检测框 | 模型路径/设备是否存在；`model.classes` 是否与模型类别匹配；深度是否对齐、min/max 深度是否合理 |
| 点击无反应 | 是否点在检测框内；C++ 是否已连接（画面左上角 `C++ CLIENT OK`）；点击处深度是否有效 |
| 坐标明显偏 | 标定文件是否为当前相机实际外参；头部角是否实读；`link_length_mm` 是否与机械一致 |
| IK/规划失败 | 目标是否在右臂工作空间；`approach_x_offset_m`、`lift_height_m` 是否过大；staging 是否碰撞桌面（调 `table` 相关仅在新版 ROS2 包，本工程需人工保证安全距离） |
| 抓到但掉落 | `grasp_yaw_deg` 与瓶身朝向是否匹配；`handClose` 角度是否足够；接近速度是否过快 |

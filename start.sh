#!/bin/bash
# bottle_grasp 启动脚本（在 AGX 机器人上运行）
# 依赖：
#   1) 视觉依赖已装到 AGX 系统 python3（见 python/requirements.txt 与 安装.txt 的 Jetson 配方）
#   2) GUI 需要桌面会话（NoMachine/本地显示器），不要用纯 ssh 终端
# 用法：
#   ./scripts/start.sh            # 前台运行（推荐，便于看日志）
#   ./scripts/start.sh --bg       # 后台运行（日志写到 logs/）

set -e
WORK_DIR="$(cd "$(dirname "$0")/.." && pwd)"
cd "$WORK_DIR"
mkdir -p logs

PYTHON="${PYTHON:-python3}"
echo "使用 Python: $PYTHON ($("$PYTHON" --version 2>&1))"
"$PYTHON" -c "import torch, ultralytics, pyrealsense2, cv2, numpy" 2>/dev/null \
    || { echo "系统 python3 视觉依赖不完整（缺 torch/ultralytics 等），请先安装："; \
         echo "  python3 -m pip install torch==2.8.0 torchvision==0.23.0 --index-url https://pypi.jetson-ai-lab.io/jp6/cu126"; \
         echo "  python3 -m pip install \"numpy<2\" opencv-python PyYAML pyrealsense2 ultralytics"; exit 1; }

MODE="${1:---fg}"

if [ ! -f build/bottle_grasp ]; then
    echo "未找到 build/bottle_grasp，请先构建："
    echo "  cd $WORK_DIR && ./scripts/build.sh"
    exit 1
fi

if [ "$MODE" = "--bg" ]; then
    echo "后台启动：视觉服务 + C++ 主控"
    nohup "$PYTHON" python/vision_server.py > logs/vision.log 2>&1 &
    echo "视觉服务 PID: $!"
    sleep 4
    sudo ./build/bottle_grasp > logs/bottle_grasp.log 2>&1 &
    echo "C++ 主控 PID: $!"
    echo "日志: logs/vision.log, logs/bottle_grasp.log"
else
    echo "前台启动：先开视觉服务(新终端)，再开 C++ 主控"
    echo "  终端1: $PYTHON python/vision_server.py"
    echo "  终端2: sudo ./build/bottle_grasp"
fi

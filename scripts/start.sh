#!/bin/bash
# bottle_grasp 启动脚本（在 AGX 机器人上运行）
# 依赖：
#   1) 视觉依赖已装好（见 python/requirements.txt 与 安装.txt 的 Jetson 配方）
#   2) GUI 需要桌面会话（NoMachine/本地显示器），不要用纯 ssh 终端
# 用法：
#   ./scripts/start.sh            # 前台运行（推荐，便于看日志）
#   ./scripts/start.sh --bg       # 后台运行（日志写到 logs/）

set -e
WORK_DIR="$(cd "$(dirname "$0")/.." && pwd)"
cd "$WORK_DIR"
mkdir -p logs

MODE="${1:---fg}"

if [ ! -f build/bottle_grasp ]; then
    echo "未找到 build/bottle_grasp，请先构建："
    echo "  cd $WORK_DIR && mkdir -p build && cd build && cmake .. && make -j\$(nproc)"
    exit 1
fi

if [ "$MODE" = "--bg" ]; then
    echo "后台启动：视觉服务 + C++ 主控"
    nohup python3 python/vision_server.py > logs/vision.log 2>&1 &
    echo "视觉服务 PID: $!"
    sleep 4
    sudo ./build/bottle_grasp > logs/bottle_grasp.log 2>&1 &
    echo "C++ 主控 PID: $!"
    echo "日志: logs/vision.log, logs/bottle_grasp.log"
else
    echo "前台启动：先开视觉服务(新终端)，再开 C++ 主控"
    echo "  终端1: python3 python/vision_server.py"
    echo "  终端2: sudo ./build/bottle_grasp"
fi

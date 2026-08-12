#!/bin/bash
# 在 AGX 机器人上构建 bottle_grasp
set -e
WORK_DIR="$(cd "$(dirname "$0")/.." && pwd)"
cd "$WORK_DIR"
mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j"$(nproc)"
echo "构建完成: $WORK_DIR/build/bottle_grasp"

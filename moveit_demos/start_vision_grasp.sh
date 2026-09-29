#!/bin/bash
# 视觉抓取启动脚本

echo "=========================================="
echo "视觉抓取蓝色纸巾 - 启动脚本"
echo "=========================================="

# 检查 RealSense 库
if ! python3 -c "import pyrealsense2" 2>/dev/null; then
    echo "错误: 未安装 pyrealsense2"
    echo "请运行: pip install pyrealsense2"
    exit 1
fi

# 检查 OpenCV
if ! python3 -c "import cv2" 2>/dev/null; then
    echo "错误: 未安装 OpenCV"
    echo "请运行: pip install opencv-python"
    exit 1
fi

# 检查 scipy
if ! python3 -c "import scipy" 2>/dev/null; then
    echo "错误: 未安装 scipy"
    echo "请运行: pip install scipy"
    exit 1
fi

echo ""
echo "✓ 依赖检查完成"
echo ""
echo "请确保已启动机械臂控制节点:"
echo "ros2 launch agx_arm_ctrl start_single_agx_arm_moveit.launch.py \\"
echo "  can_port:=can0 arm_type:=piper effector_type:=agx_gripper"
echo ""

# 获取脚本所在目录
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# 运行 Python 脚本
python3 "${SCRIPT_DIR}/vision_grasp_blue_tissue.py"

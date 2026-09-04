#!/bin/bash

# Gazebo 仿真快速启动脚本

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# 打印彩色信息
print_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# 显示标题
clear
echo -e "${CYAN}"
echo "=========================================="
echo "   MoveIt + Gazebo 仿真快速启动"
echo "=========================================="
echo -e "${NC}"

# 检查 ROS2 环境
print_info "检查 ROS2 环境..."

if [ -z "$ROS_DISTRO" ]; then
    print_error "ROS2 环境未设置！"
    print_info "请先运行: source /opt/ros/humble/setup.bash"
    exit 1
fi

print_success "ROS2 环境: $ROS_DISTRO"

# 检查工作空间
WORKSPACE_DIR="$HOME/agx_arm_ws"
if [ ! -d "$WORKSPACE_DIR" ]; then
    print_error "工作空间未找到: $WORKSPACE_DIR"
    exit 1
fi

print_success "工作空间: $WORKSPACE_DIR"

# Source 工作空间
print_info "加载工作空间环境..."
source "$WORKSPACE_DIR/install/setup.bash"

# 检查 Gazebo
print_info "检查 Gazebo 安装..."
if ! command -v gazebo &> /dev/null; then
    print_error "Gazebo 未安装！"
    echo ""
    echo "请安装 Gazebo:"
    echo "  sudo apt install ros-$ROS_DISTRO-gazebo-ros-pkgs ros-$ROS_DISTRO-gazebo-ros2-control"
    exit 1
fi

GAZEBO_VERSION=$(gazebo --version | head -n1)
print_success "Gazebo: $GAZEBO_VERSION"

# 选择机械臂型号
echo ""
echo "=========================================="
echo "请选择机械臂型号:"
echo "=========================================="
echo "1. Piper"
echo "2. Piper X"
echo "3. Piper H"
echo "4. Piper L"
echo "5. Nero"
echo "=========================================="
read -p "请输入选项 (1-5): " arm_choice

case $arm_choice in
    1) ARM_TYPE="piper" ;;
    2) ARM_TYPE="piper_x" ;;
    3) ARM_TYPE="piper_h" ;;
    4) ARM_TYPE="piper_l" ;;
    5) ARM_TYPE="nero" ;;
    *) 
        print_error "无效选项"
        exit 1
        ;;
esac

print_success "已选择: $ARM_TYPE"

# 选择末端执行器
echo ""
echo "=========================================="
echo "请选择末端执行器:"
echo "=========================================="
echo "1. 无末端执行器 (none)"
echo "2. AgileX 夹爪 (agx_gripper)"
echo "3. Revo2 灵巧手 (revo2)"
echo "=========================================="
read -p "请输入选项 (1-3): " effector_choice

case $effector_choice in
    1) EFFECTOR_TYPE="none" ;;
    2) EFFECTOR_TYPE="agx_gripper" ;;
    3) EFFECTOR_TYPE="revo2" ;;
    *) 
        print_error "无效选项"
        exit 1
        ;;
esac

print_success "已选择: $EFFECTOR_TYPE"

# 如果是灵巧手，选择左右手
REVO2_TYPE=""
if [ "$EFFECTOR_TYPE" == "revo2" ]; then
    echo ""
    echo "=========================================="
    echo "请选择灵巧手类型:"
    echo "=========================================="
    echo "1. 左手 (left)"
    echo "2. 右手 (right)"
    echo "=========================================="
    read -p "请输入选项 (1-2): " hand_choice
    
    case $hand_choice in
        1) REVO2_TYPE="left" ;;
        2) REVO2_TYPE="right" ;;
        *) 
            print_error "无效选项"
            exit 1
            ;;
    esac
    
    print_success "已选择: $REVO2_TYPE"
fi

# 选择启动模式
echo ""
echo "=========================================="
echo "请选择启动模式:"
echo "=========================================="
echo "1. Gazebo + MoveIt + RViz (完整可视化)"
echo "2. Gazebo + MoveIt (无 RViz，轻量级)"
echo "3. Gazebo 无 GUI (服务器模式)"
echo "=========================================="
read -p "请输入选项 (1-3): " mode_choice

USE_RVIZ="true"
GAZEBO_GUI="true"

case $mode_choice in
    1) 
        USE_RVIZ="true"
        GAZEBO_GUI="true"
        ;;
    2) 
        USE_RVIZ="false"
        GAZEBO_GUI="true"
        ;;
    3) 
        USE_RVIZ="false"
        GAZEBO_GUI="false"
        ;;
    *) 
        print_error "无效选项"
        exit 1
        ;;
esac

# 构建启动命令
echo ""
echo "=========================================="
print_info "准备启动 Gazebo 仿真..."
echo "=========================================="

LAUNCH_CMD="ros2 launch agx_arm_moveit demo.launch.py"
LAUNCH_CMD="$LAUNCH_CMD arm_type:=$ARM_TYPE"
LAUNCH_CMD="$LAUNCH_CMD effector_type:=$EFFECTOR_TYPE"
LAUNCH_CMD="$LAUNCH_CMD use_sim_time:=true"
LAUNCH_CMD="$LAUNCH_CMD use_rviz:=$USE_RVIZ"
LAUNCH_CMD="$LAUNCH_CMD gui:=$GAZEBO_GUI"

if [ -n "$REVO2_TYPE" ]; then
    LAUNCH_CMD="$LAUNCH_CMD revo2_type:=$REVO2_TYPE"
fi

echo ""
print_info "启动命令:"
echo -e "${CYAN}$LAUNCH_CMD${NC}"
echo ""

print_info "启动信息:"
echo "  - 机械臂: $ARM_TYPE"
echo "  - 末端执行器: $EFFECTOR_TYPE"
if [ -n "$REVO2_TYPE" ]; then
    echo "  - 灵巧手: $REVO2_TYPE"
fi
echo "  - RViz: $USE_RVIZ"
echo "  - Gazebo GUI: $GAZEBO_GUI"
echo ""

print_warning "Gazebo 首次启动可能需要下载模型，请耐心等待..."
print_info "启动后，在新终端运行: python3 moveit_gazebo_demo.py"
echo ""

read -p "按 Enter 键继续启动，或 Ctrl+C 取消... "

echo ""
print_success "启动 Gazebo 仿真环境..."
echo ""

# 执行启动命令
eval $LAUNCH_CMD

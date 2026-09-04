#!/bin/bash

# MoveIt Demo 快速启动脚本

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
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
echo "=========================================="
echo "   MoveIt Demo 快速启动脚本"
echo "=========================================="
echo ""

# 检查环境
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
echo "4. Revo2 Pro 灵巧手 (revo2_pro)"
echo "5. Revo2 Touch 灵巧手 (revo2_touch)"
echo "=========================================="
read -p "请输入选项 (1-5): " effector_choice

case $effector_choice in
    1) EFFECTOR_TYPE="none" ;;
    2) EFFECTOR_TYPE="agx_gripper" ;;
    3) EFFECTOR_TYPE="revo2" ;;
    4) EFFECTOR_TYPE="revo2_pro" ;;
    5) EFFECTOR_TYPE="revo2_touch" ;;
    *) 
        print_error "无效选项"
        exit 1
        ;;
esac

print_success "已选择: $EFFECTOR_TYPE"

# 如果是灵巧手，选择左右手
REVO2_TYPE=""
if [[ "$EFFECTOR_TYPE" == "revo2"* ]]; then
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
echo "1. 仅 MoveIt (仿真模式，无需真实机械臂)"
echo "2. 真实机械臂 + MoveIt (需要连接真实机械臂)"
echo "=========================================="
read -p "请输入选项 (1-2): " mode_choice

case $mode_choice in
    1) MODE="simulation" ;;
    2) MODE="real" ;;
    *) 
        print_error "无效选项"
        exit 1
        ;;
esac

# 如果是真实机械臂模式，询问 CAN 端口
CAN_PORT=""
if [ "$MODE" == "real" ]; then
    echo ""
    print_info "真实机械臂模式需要 CAN 端口"
    read -p "请输入 CAN 端口 (默认: can0): " user_can_port
    CAN_PORT=${user_can_port:-can0}
    print_success "CAN 端口: $CAN_PORT"
fi

# 构建启动命令
echo ""
echo "=========================================="
print_info "准备启动..."
echo "=========================================="

if [ "$MODE" == "simulation" ]; then
    # 仿真模式
    LAUNCH_CMD="ros2 launch agx_arm_moveit demo.launch.py arm_type:=$ARM_TYPE effector_type:=$EFFECTOR_TYPE"
    
    if [ -n "$REVO2_TYPE" ]; then
        LAUNCH_CMD="$LAUNCH_CMD revo2_type:=$REVO2_TYPE"
    fi
    
    print_info "启动命令:"
    echo "$LAUNCH_CMD"
    echo ""
    
    print_warning "这是仿真模式，不会控制真实机械臂"
    print_info "按 Ctrl+C 退出"
    echo ""
    
    sleep 2
    eval $LAUNCH_CMD
    
else
    # 真实机械臂模式
    LAUNCH_CMD="ros2 launch agx_arm_ctrl start_single_agx_arm_moveit.launch.py can_port:=$CAN_PORT arm_type:=$ARM_TYPE effector_type:=$EFFECTOR_TYPE"
    
    if [ -n "$REVO2_TYPE" ]; then
        LAUNCH_CMD="$LAUNCH_CMD revo2_type:=$REVO2_TYPE"
    fi
    
    print_info "启动命令:"
    echo "$LAUNCH_CMD"
    echo ""
    
    print_warning "即将控制真实机械臂！"
    print_warning "请确保:"
    print_warning "  1. CAN 模块已激活"
    print_warning "  2. 机械臂周围安全"
    print_warning "  3. 急停按钮可用"
    echo ""
    
    read -p "按 Enter 键继续，或 Ctrl+C 取消... "
    
    eval $LAUNCH_CMD
fi

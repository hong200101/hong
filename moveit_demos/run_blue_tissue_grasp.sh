#!/bin/bash

###############################################################################
# 蓝色纸巾包视觉抓取系统启动脚本
# 用法: ./run_blue_tissue_grasp.sh [选项]
###############################################################################

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 脚本目录
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
CONFIG_FILE="${SCRIPT_DIR}/config/blue_tissue_grasp_config.yaml"

# 打印标题
print_header() {
    echo -e "${BLUE}======================================================================${NC}"
    echo -e "${BLUE}           蓝色纸巾包视觉抓取系统启动脚本${NC}"
    echo -e "${BLUE}======================================================================${NC}"
}

# 打印信息
print_info() {
    echo -e "${GREEN}[信息]${NC} $1"
}

# 打印警告
print_warning() {
    echo -e "${YELLOW}[警告]${NC} $1"
}

# 打印错误
print_error() {
    echo -e "${RED}[错误]${NC} $1"
}

# 检查依赖
check_dependencies() {
    print_info "检查依赖..."
    
    # 检查 Python 模块
    python3 -c "import pyrealsense2" 2>/dev/null
    if [ $? -ne 0 ]; then
        print_error "未安装 pyrealsense2"
        print_info "安装命令: pip3 install pyrealsense2"
        return 1
    fi
    
    python3 -c "import cv2" 2>/dev/null
    if [ $? -ne 0 ]; then
        print_error "未安装 opencv-python"
        print_info "安装命令: pip3 install opencv-python"
        return 1
    fi
    
    python3 -c "import numpy" 2>/dev/null
    if [ $? -ne 0 ]; then
        print_error "未安装 numpy"
        print_info "安装命令: pip3 install numpy"
        return 1
    fi
    
    python3 -c "import yaml" 2>/dev/null
    if [ $? -ne 0 ]; then
        print_error "未安装 pyyaml"
        print_info "安装命令: pip3 install pyyaml"
        return 1
    fi
    
    print_info "✓ 所有依赖已安装"
    return 0
}

# 显示帮助
show_help() {
    echo "用法: $0 [选项]"
    echo ""
    echo "选项:"
    echo "  -h, --help              显示此帮助信息"
    echo "  -c, --config <file>     指定配置文件 (默认: config/blue_tissue_grasp_config.yaml)"
    echo "  -a, --auto              自动运行模式（非交互）"
    echo "  -p, --place             抓取后放置物体"
    echo "  -i, --interactive       交互式模式（默认）"
    echo "  --no-viz                不显示可视化"
    echo ""
    echo "示例:"
    echo "  $0                      # 交互式模式"
    echo "  $0 -a                   # 自动运行一次抓取"
    echo "  $0 -a -p                # 自动运行抓取并放置"
    echo "  $0 -c custom.yaml       # 使用自定义配置"
    echo ""
}

# 主函数
main() {
    print_header
    
    # 解析命令行参数
    AUTO_MODE=""
    PLACE_MODE=""
    VIZ_MODE="--visualize"
    CUSTOM_CONFIG=""
    
    while [[ $# -gt 0 ]]; do
        case $1 in
            -h|--help)
                show_help
                exit 0
                ;;
            -c|--config)
                CUSTOM_CONFIG="--config $2"
                shift 2
                ;;
            -a|--auto)
                AUTO_MODE="--auto"
                shift
                ;;
            -p|--place)
                PLACE_MODE="--place"
                shift
                ;;
            -i|--interactive)
                AUTO_MODE=""
                shift
                ;;
            --no-viz)
                VIZ_MODE=""
                shift
                ;;
            *)
                print_error "未知选项: $1"
                show_help
                exit 1
                ;;
        esac
    done
    
    # 检查依赖
    check_dependencies
    if [ $? -ne 0 ]; then
        print_error "依赖检查失败，请先安装缺失的依赖"
        exit 1
    fi
    
    # 检查配置文件
    if [ -n "$CUSTOM_CONFIG" ]; then
        CONFIG_TO_USE=$(echo $CUSTOM_CONFIG | awk '{print $2}')
    else
        CONFIG_TO_USE="$CONFIG_FILE"
    fi
    
    if [ ! -f "$CONFIG_TO_USE" ]; then
        print_warning "配置文件不存在: $CONFIG_TO_USE"
        print_info "将使用默认配置"
    else
        print_info "使用配置文件: $CONFIG_TO_USE"
    fi
    
    # 显示运行模式
    echo ""
    if [ -n "$AUTO_MODE" ]; then
        print_info "运行模式: 自动"
        if [ -n "$PLACE_MODE" ]; then
            print_info "  - 抓取后放置物体"
        fi
    else
        print_info "运行模式: 交互式"
    fi
    
    # 检查机械臂控制节点
    echo ""
    print_warning "请确保已启动机械臂控制节点!"
    echo ""
    echo "  终端1: 启动机械臂控制（带 MoveIt）"
    echo "  ros2 launch agx_arm_ctrl start_single_agx_arm_moveit.launch.py \\"
    echo "    can_port:=can0 arm_type:=piper effector_type:=agx_gripper"
    echo ""
    
    read -p "机械臂控制节点已启动? (y/n) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        print_error "请先启动机械臂控制节点"
        exit 1
    fi
    
    # 启动视觉抓取系统
    print_info "启动视觉抓取系统..."
    echo ""
    
    cd "$SCRIPT_DIR"
    python3 blue_tissue_grasp.py $CUSTOM_CONFIG $AUTO_MODE $PLACE_MODE $VIZ_MODE
    
    # 检查退出状态
    if [ $? -eq 0 ]; then
        echo ""
        print_info "✓ 系统正常退出"
    else
        echo ""
        print_error "✗ 系统异常退出"
    fi
}

# 运行主函数
main "$@"

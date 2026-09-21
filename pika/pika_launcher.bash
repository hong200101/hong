#!/bin/bash

# ==============================================================================
# Pika 一键启动脚本
# 支持所有设备模式的快速启动、配置和数据处理
# ==============================================================================

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
MAGENTA='\033[0;35m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color
BOLD='\033[1m'

# 获取脚本所在目录
SCRIPT_DIR=$(dirname "$(readlink -f "$0")")
cd "$SCRIPT_DIR"

# ==============================================================================
# 工具函数
# ==============================================================================

print_header() {
    echo -e "\n${CYAN}========================================${NC}"
    echo -e "${BOLD}$1${NC}"
    echo -e "${CYAN}========================================${NC}\n"
}

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

print_step() {
    echo -e "${MAGENTA}[STEP $1]${NC} $2"
}

press_any_key() {
    echo -e "\n${YELLOW}按任意键继续...${NC}"
    read -n 1 -s -r
}

check_ros2_environment() {
    if [ -z "$ROS_DISTRO" ]; then
        print_error "ROS2 环境未加载！"
        print_info "正在加载 ROS2 环境..."
        source /opt/ros/humble/setup.bash 2>/dev/null
        if [ $? -ne 0 ]; then
            print_error "无法加载 ROS2 环境，请确保已安装 ROS2 Humble"
            return 1
        fi
    fi
    
    if [ -f "$SCRIPT_DIR/install/setup.bash" ]; then
        source "$SCRIPT_DIR/install/setup.bash"
        print_success "ROS2 工作空间已加载"
    else
        print_warning "工作空间未构建，部分功能可能不可用"
        print_info "建议运行: colcon build --symlink-install"
    fi
    return 0
}

check_devices() {
    local device_type=$1
    local required_devices=()
    
    case $device_type in
        "single_sensor"|"single_gripper")
            required_devices=("ttyUSB50" "video50")
            ;;
        "multi_sensor"|"multi_gripper")
            required_devices=("ttyUSB50" "ttyUSB51" "video50" "video51")
            ;;
        "sensor_gripper")
            required_devices=("ttyUSB50" "ttyUSB60" "video60")
            ;;
        "helmet")
            required_devices=("ttyUSB70" "video70")
            ;;
        "multi_sensor_helmet")
            required_devices=("ttyUSB50" "ttyUSB51" "ttyUSB70" "video50" "video51" "video70")
            ;;
    esac
    
    local missing_devices=()
    for dev in "${required_devices[@]}"; do
        if [ ! -e "/dev/$dev" ]; then
            missing_devices+=("$dev")
        fi
    done
    
    if [ ${#missing_devices[@]} -gt 0 ]; then
        print_warning "以下设备未找到: ${missing_devices[*]}"
        print_info "请确保设备已连接并已完成配置"
        return 1
    fi
    
    print_success "所有设备检测通过"
    return 0
}

# ==============================================================================
# 主菜单
# ==============================================================================

show_main_menu() {
    clear
    print_header "🚀 Pika 一键启动系统"
    
    echo -e "${BOLD}请选择操作模式：${NC}\n"
    
    echo -e "${GREEN}━━━ 设备配置 ━━━${NC}"
    echo "  1) 配置设备（首次使用必选）"
    echo "  2) 检查设备状态"
    echo ""
    
    echo -e "${CYAN}━━━ 数据采集模式 ━━━${NC}"
    echo "  3) 单个 Sensor（手持夹爪）"
    echo "  4) 单个 Gripper（机械臂夹爪）"
    echo "  5) 双 Sensor（双手持夹爪）"
    echo "  6) 双 Gripper（双机械臂夹爪）"
    echo "  7) Sensor + Gripper（混合模式）"
    echo "  8) Helmet（头盔模式）"
    echo "  9) 双 Sensor + Helmet（完整模式）"
    echo ""
    
    echo -e "${YELLOW}━━━ 遥操作模式 ━━━${NC}"
    echo " 10) 单 Sensor + 遥操作"
    echo " 11) 双 Sensor + 遥操作"
    echo ""
    
    echo -e "${MAGENTA}━━━ 数据处理 ━━━${NC}"
    echo " 12) 数据同步（时间对齐）"
    echo " 13) 数据转换（生成 HDF5）"
    echo " 14) 数据加载示例"
    echo " 15) 批量处理（同步+转换）"
    echo ""
    
    echo -e "${BLUE}━━━ 工具功能 ━━━${NC}"
    echo " 16) 启动 RViz 可视化"
    echo " 17) 查找 USB 相机"
    echo " 18) 相机标定"
    echo " 19) 点云过滤"
    echo ""
    
    echo " 0) 退出"
    echo ""
    
    echo -ne "${BOLD}请输入选项 [0-19]: ${NC}"
    read -r choice
    echo ""
    
    return $choice
}

# ==============================================================================
# 设备配置
# ==============================================================================

configure_devices() {
    print_header "📋 设备配置向导"
    
    if [ ! -f "scripts/setup_device.py" ]; then
        print_error "setup_device.py 不存在！"
        press_any_key
        return
    fi
    
    print_info "启动设备配置脚本..."
    print_warning "请按照提示插拔设备"
    echo ""
    
    cd scripts
    python3 setup_device.py
    cd "$SCRIPT_DIR"
    
    print_success "设备配置完成！"
    press_any_key
}

check_device_status() {
    print_header "🔍 设备状态检查"
    
    print_step "1/4" "检查 RealSense 深度相机..."
    if command -v rs-enumerate-devices &> /dev/null; then
        rs-enumerate-devices -s 2>/dev/null | grep -E "Serial Number|Device Name" || print_warning "未检测到 RealSense 设备"
    else
        print_warning "rs-enumerate-devices 命令不存在，请安装 librealsense2"
    fi
    echo ""
    
    print_step "2/4" "检查串口设备..."
    if ls /dev/ttyUSB* &> /dev/null; then
        ls -l /dev/ttyUSB* | awk '{print $NF}'
    else
        print_warning "未检测到 ttyUSB 设备"
    fi
    echo ""
    
    print_step "3/4" "检查视频设备..."
    if ls /dev/video* &> /dev/null; then
        for video in /dev/video*; do
            if [ -c "$video" ]; then
                echo "$video: $(v4l2-ctl --device=$video --all 2>/dev/null | grep 'Card type' | cut -d: -f2 | xargs)"
            fi
        done
    else
        print_warning "未检测到 video 设备"
    fi
    echo ""
    
    print_step "4/4" "检查 Vive Tracker（定位标签）..."
    if lsusb | grep -q "28de:2300"; then
        print_success "检测到 Vive Tracker"
        lsusb | grep "28de:2300"
    else
        print_info "未检测到 Vive Tracker（可选设备）"
    fi
    
    press_any_key
}

# ==============================================================================
# 数据采集模式启动
# ==============================================================================

launch_single_sensor() {
    print_header "🎯 启动单个 Sensor"
    
    check_devices "single_sensor"
    if [ $? -ne 0 ]; then
        print_error "设备检查失败，是否继续？(y/n)"
        read -r continue
        if [ "$continue" != "y" ]; then
            return
        fi
    fi
    
    check_ros2_environment || return
    
    print_info "启动单个 Sensor 节点..."
    cd scripts
    bash start_single_sensor.bash
}

launch_single_gripper() {
    print_header "🎯 启动单个 Gripper"
    
    check_devices "single_gripper"
    if [ $? -ne 0 ]; then
        print_error "设备检查失败，是否继续？(y/n)"
        read -r continue
        if [ "$continue" != "y" ]; then
            return
        fi
    fi
    
    check_ros2_environment || return
    
    print_info "启动单个 Gripper 节点..."
    cd scripts
    bash start_single_gripper.bash
}

launch_multi_sensor() {
    print_header "🎯 启动双 Sensor"
    
    check_devices "multi_sensor"
    if [ $? -ne 0 ]; then
        print_error "设备检查失败，是否继续？(y/n)"
        read -r continue
        if [ "$continue" != "y" ]; then
            return
        fi
    fi
    
    check_ros2_environment || return
    
    echo -ne "${BOLD}是否指定命名空间？(y/n): ${NC}"
    read -r use_namespace
    
    if [ "$use_namespace" = "y" ]; then
        echo -ne "请输入命名空间名称: "
        read -r namespace
        print_info "启动双 Sensor 节点（命名空间: $namespace）..."
        cd scripts
        bash start_multi_sensor.bash "$namespace"
    else
        print_info "启动双 Sensor 节点..."
        cd scripts
        bash start_multi_sensor.bash
    fi
}

launch_multi_gripper() {
    print_header "🎯 启动双 Gripper"
    
    check_devices "multi_gripper"
    if [ $? -ne 0 ]; then
        print_error "设备检查失败，是否继续？(y/n)"
        read -r continue
        if [ "$continue" != "y" ]; then
            return
        fi
    fi
    
    check_ros2_environment || return
    
    echo -ne "${BOLD}是否指定命名空间？(y/n): ${NC}"
    read -r use_namespace
    
    if [ "$use_namespace" = "y" ]; then
        echo -ne "请输入命名空间名称: "
        read -r namespace
        print_info "启动双 Gripper 节点（命名空间: $namespace）..."
        cd scripts
        bash start_multi_gripper.bash "$namespace"
    else
        print_info "启动双 Gripper 节点..."
        cd scripts
        bash start_multi_gripper.bash
    fi
}

launch_sensor_gripper() {
    print_header "🎯 启动 Sensor + Gripper"
    
    check_devices "sensor_gripper"
    if [ $? -ne 0 ]; then
        print_error "设备检查失败，是否继续？(y/n)"
        read -r continue
        if [ "$continue" != "y" ]; then
            return
        fi
    fi
    
    check_ros2_environment || return
    
    print_info "启动 Sensor + Gripper 节点..."
    cd scripts
    bash start_sensor_gripper.bash
}

launch_helmet() {
    print_header "🎯 启动 Helmet"
    
    check_devices "helmet"
    if [ $? -ne 0 ]; then
        print_error "设备检查失败，是否继续？(y/n)"
        read -r continue
        if [ "$continue" != "y" ]; then
            return
        fi
    fi
    
    check_ros2_environment || return
    
    print_info "启动 Helmet 节点..."
    cd scripts
    if [ -f "start_helmet.bash" ]; then
        bash start_helmet.bash
    else
        print_error "start_helmet.bash 不存在"
        press_any_key
    fi
}

launch_multi_sensor_helmet() {
    print_header "🎯 启动双 Sensor + Helmet"
    
    check_devices "multi_sensor_helmet"
    if [ $? -ne 0 ]; then
        print_error "设备检查失败，是否继续？(y/n)"
        read -r continue
        if [ "$continue" != "y" ]; then
            return
        fi
    fi
    
    check_ros2_environment || return
    
    echo -ne "${BOLD}是否指定命名空间？(y/n): ${NC}"
    read -r use_namespace
    
    if [ "$use_namespace" = "y" ]; then
        echo -ne "请输入命名空间名称: "
        read -r namespace
        print_info "启动双 Sensor + Helmet 节点（命名空间: $namespace）..."
        cd scripts
        bash start_multi_sensor_helmet_whit_tracker.bash "$namespace"
    else
        print_info "启动双 Sensor + Helmet 节点..."
        cd scripts
        bash start_multi_sensor_helmet_whit_tracker.bash
    fi
}

launch_single_sensor_teleop() {
    print_header "🎮 启动单 Sensor + 遥操作"
    
    check_devices "single_sensor"
    if [ $? -ne 0 ]; then
        print_warning "设备检查失败，但可以继续启动遥操作模式"
    fi
    
    check_ros2_environment || return
    
    print_info "启动单 Sensor + 遥操作节点..."
    cd scripts
    bash start_single_sensor_whit_teleop.bash
}

launch_multi_sensor_teleop() {
    print_header "🎮 启动双 Sensor + 遥操作"
    
    check_devices "multi_sensor"
    if [ $? -ne 0 ]; then
        print_warning "设备检查失败，但可以继续启动遥操作模式"
    fi
    
    check_ros2_environment || return
    
    print_info "启动双 Sensor + 遥操作节点..."
    cd scripts
    bash start_multi_sensor_whit_teleop.bash
}

# ==============================================================================
# 数据处理
# ==============================================================================

data_sync() {
    print_header "🔄 数据同步"
    
    echo -ne "${BOLD}数据集目录 [默认: ~/data]: ${NC}"
    read -r dataset_dir
    dataset_dir=${dataset_dir:-~/data}
    dataset_dir=$(eval echo "$dataset_dir")
    
    if [ ! -d "$dataset_dir" ]; then
        print_error "目录不存在: $dataset_dir"
        press_any_key
        return
    fi
    
    echo ""
    print_info "可用的 episodes:"
    ls -1 "$dataset_dir" | grep -v ".tar.gz" | grep -v ".hdf5" | nl
    echo ""
    
    echo -ne "${BOLD}Episode 名称 [留空处理所有]: ${NC}"
    read -r episode_name
    
    echo -ne "${BOLD}数据类型 (sensor/gripper/aloha) [默认: gripper]: ${NC}"
    read -r data_type
    data_type=${data_type:-gripper}
    
    echo -ne "${BOLD}时间差阈值（秒） [默认: 0.05]: ${NC}"
    read -r time_diff
    time_diff=${time_diff:-0.05}
    
    print_info "开始数据同步..."
    cd scripts
    
    if [ -z "$episode_name" ]; then
        # 处理所有 episodes
        for episode in "$dataset_dir"/*; do
            if [ -d "$episode" ] && [ ! -f "$episode.tar.gz" ]; then
                episode_name=$(basename "$episode")
                print_step "处理" "$episode_name"
                python3 data_sync.py \
                    --datasetDir "$dataset_dir" \
                    --episodeName "$episode_name" \
                    --type "$data_type" \
                    --timeDiffLimit "$time_diff"
            fi
        done
    else
        python3 data_sync.py \
            --datasetDir "$dataset_dir" \
            --episodeName "$episode_name" \
            --type "$data_type" \
            --timeDiffLimit "$time_diff"
    fi
    
    cd "$SCRIPT_DIR"
    print_success "数据同步完成！"
    press_any_key
}

data_to_hdf5() {
    print_header "💾 数据转换为 HDF5"
    
    echo -ne "${BOLD}数据集目录 [默认: ~/data]: ${NC}"
    read -r dataset_dir
    dataset_dir=${dataset_dir:-~/data}
    dataset_dir=$(eval echo "$dataset_dir")
    
    if [ ! -d "$dataset_dir" ]; then
        print_error "目录不存在: $dataset_dir"
        press_any_key
        return
    fi
    
    echo ""
    print_info "可用的 episodes:"
    ls -1 "$dataset_dir" | grep -v ".tar.gz" | grep -v ".hdf5" | nl
    echo ""
    
    echo -ne "${BOLD}Episode 名称 [留空处理所有]: ${NC}"
    read -r episode_name
    
    echo -ne "${BOLD}数据类型 (sensor/gripper/aloha) [默认: gripper]: ${NC}"
    read -r data_type
    data_type=${data_type:-gripper}
    
    echo -ne "${BOLD}使用索引模式（节省空间）？(y/n) [默认: y]: ${NC}"
    read -r use_index
    use_index=${use_index:-y}
    use_index_bool="True"
    if [ "$use_index" != "y" ]; then
        use_index_bool="False"
    fi
    
    echo -ne "${BOLD}包含点云数据？(y/n) [默认: n]: ${NC}"
    read -r use_pointcloud
    use_pointcloud=${use_pointcloud:-n}
    use_pointcloud_bool="False"
    if [ "$use_pointcloud" = "y" ]; then
        use_pointcloud_bool="True"
    fi
    
    print_info "开始数据转换..."
    cd scripts
    
    if [ -z "$episode_name" ]; then
        python3 data_to_hdf5.py \
            --datasetDir "$dataset_dir" \
            --type "$data_type" \
            --useIndex "$use_index_bool" \
            --useCameraPointCloud "$use_pointcloud_bool"
    else
        python3 data_to_hdf5.py \
            --datasetDir "$dataset_dir" \
            --episodeName "$episode_name" \
            --type "$data_type" \
            --useIndex "$use_index_bool" \
            --useCameraPointCloud "$use_pointcloud_bool"
    fi
    
    cd "$SCRIPT_DIR"
    print_success "数据转换完成！"
    press_any_key
}

data_load_example() {
    print_header "📊 数据加载示例"
    
    echo -ne "${BOLD}数据集目录 [默认: ~/data]: ${NC}"
    read -r dataset_dir
    dataset_dir=${dataset_dir:-~/data}
    dataset_dir=$(eval echo "$dataset_dir")
    
    if [ ! -d "$dataset_dir" ]; then
        print_error "目录不存在: $dataset_dir"
        press_any_key
        return
    fi
    
    echo -ne "${BOLD}Batch Size [默认: 16]: ${NC}"
    read -r batch_size
    batch_size=${batch_size:-16}
    
    print_info "运行数据加载示例..."
    cd scripts
    python3 load_data_example.py \
        --datasetDir "$dataset_dir" \
        --batchSize "$batch_size"
    
    cd "$SCRIPT_DIR"
    press_any_key
}

data_batch_process() {
    print_header "⚡ 批量处理（同步 + 转换）"
    
    echo -ne "${BOLD}数据集目录 [默认: ~/data]: ${NC}"
    read -r dataset_dir
    dataset_dir=${dataset_dir:-~/data}
    dataset_dir=$(eval echo "$dataset_dir")
    
    if [ ! -d "$dataset_dir" ]; then
        print_error "目录不存在: $dataset_dir"
        press_any_key
        return
    fi
    
    echo -ne "${BOLD}数据类型 (sensor/gripper/aloha) [默认: gripper]: ${NC}"
    read -r data_type
    data_type=${data_type:-gripper}
    
    print_info "开始批量处理..."
    cd scripts
    
    # 遍历所有 episodes
    for episode in "$dataset_dir"/*; do
        if [ -d "$episode" ] && [ ! -f "$episode.tar.gz" ]; then
            episode_name=$(basename "$episode")
            
            print_step "1/2" "同步 $episode_name"
            python3 data_sync.py \
                --datasetDir "$dataset_dir" \
                --episodeName "$episode_name" \
                --type "$data_type" \
                --timeDiffLimit 0.05
            
            print_step "2/2" "转换 $episode_name"
            python3 data_to_hdf5.py \
                --datasetDir "$dataset_dir" \
                --episodeName "$episode_name" \
                --type "$data_type" \
                --useIndex True \
                --useCameraPointCloud False
            
            print_success "$episode_name 处理完成"
            echo ""
        fi
    done
    
    cd "$SCRIPT_DIR"
    print_success "批量处理完成！"
    press_any_key
}

# ==============================================================================
# 工具功能
# ==============================================================================

launch_rviz() {
    print_header "👁️ 启动 RViz 可视化"
    
    check_ros2_environment || return
    
    print_info "启动 RViz2..."
    rviz2 &
    
    print_success "RViz2 已在后台启动"
    press_any_key
}

find_usb_camera() {
    print_header "🔍 查找 USB 相机"
    
    if [ ! -f "scripts/find_usb_camera.py" ]; then
        print_error "find_usb_camera.py 不存在！"
        press_any_key
        return
    fi
    
    cd scripts
    python3 find_usb_camera.py
    cd "$SCRIPT_DIR"
    
    press_any_key
}

camera_calibration() {
    print_header "📐 相机标定"
    
    print_info "相机标定功能"
    print_warning "此功能需要打印标定板"
    print_info "标定板可以从 ROS 官方下载"
    echo ""
    
    if [ -f "scripts/calibration.bash" ]; then
        cd scripts
        bash calibration.bash
        cd "$SCRIPT_DIR"
    else
        print_error "calibration.bash 不存在"
    fi
    
    press_any_key
}

point_cloud_filter() {
    print_header "☁️ 点云过滤"
    
    if [ ! -f "scripts/camera_point_cloud_filter.py" ]; then
        print_error "camera_point_cloud_filter.py 不存在！"
        press_any_key
        return
    fi
    
    echo -ne "${BOLD}输入点云文件路径: ${NC}"
    read -r pointcloud_file
    
    if [ ! -f "$pointcloud_file" ]; then
        print_error "文件不存在: $pointcloud_file"
        press_any_key
        return
    fi
    
    cd scripts
    python3 camera_point_cloud_filter.py "$pointcloud_file"
    cd "$SCRIPT_DIR"
    
    press_any_key
}

# ==============================================================================
# 主循环
# ==============================================================================

main() {
    while true; do
        show_main_menu
        choice=$?
        
        case $choice in
            1) configure_devices ;;
            2) check_device_status ;;
            3) launch_single_sensor ;;
            4) launch_single_gripper ;;
            5) launch_multi_sensor ;;
            6) launch_multi_gripper ;;
            7) launch_sensor_gripper ;;
            8) launch_helmet ;;
            9) launch_multi_sensor_helmet ;;
            10) launch_single_sensor_teleop ;;
            11) launch_multi_sensor_teleop ;;
            12) data_sync ;;
            13) data_to_hdf5 ;;
            14) data_load_example ;;
            15) data_batch_process ;;
            16) launch_rviz ;;
            17) find_usb_camera ;;
            18) camera_calibration ;;
            19) point_cloud_filter ;;
            0)
                print_header "👋 感谢使用 Pika 一键启动系统"
                exit 0
                ;;
            *)
                print_error "无效选项，请重新选择"
                sleep 1
                ;;
        esac
    done
}

# 运行主程序
main

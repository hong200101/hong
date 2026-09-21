#!/bin/bash

# ==============================================================================
# Pika Launcher 安装脚本
# 用于设置 pika_launcher.bash 并添加全局快捷命令
# ==============================================================================

SCRIPT_DIR=$(dirname "$(readlink -f "$0")")
LAUNCHER_PATH="$SCRIPT_DIR/pika_launcher.bash"

# 颜色定义
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

echo -e "${BLUE}================================================${NC}"
echo -e "${BLUE}   Pika Launcher 安装脚本${NC}"
echo -e "${BLUE}================================================${NC}\n"

# 1. 检查文件存在
if [ ! -f "$LAUNCHER_PATH" ]; then
    echo -e "${RED}错误: pika_launcher.bash 不存在！${NC}"
    exit 1
fi

# 2. 赋予执行权限
echo -e "${BLUE}[1/3]${NC} 赋予执行权限..."
chmod +x "$LAUNCHER_PATH"
if [ $? -eq 0 ]; then
    echo -e "${GREEN}✓ 执行权限设置成功${NC}\n"
else
    echo -e "${RED}✗ 执行权限设置失败${NC}\n"
    exit 1
fi

# 3. 添加别名到 .bashrc
echo -e "${BLUE}[2/3]${NC} 添加快捷命令到 ~/.bashrc..."

ALIAS_LINE="alias pika='cd $SCRIPT_DIR && ./pika_launcher.bash'"
BASHRC="$HOME/.bashrc"

# 检查别名是否已存在
if grep -Fxq "$ALIAS_LINE" "$BASHRC" 2>/dev/null; then
    echo -e "${YELLOW}⚠ 快捷命令已存在${NC}\n"
else
    echo "" >> "$BASHRC"
    echo "# Pika Launcher 快捷命令" >> "$BASHRC"
    echo "$ALIAS_LINE" >> "$BASHRC"
    echo -e "${GREEN}✓ 快捷命令添加成功${NC}\n"
fi

# 4. 立即生效
echo -e "${BLUE}[3/3]${NC} 使配置生效..."
source "$BASHRC"
echo -e "${GREEN}✓ 配置已生效${NC}\n"

# 显示使用说明
echo -e "${GREEN}================================================${NC}"
echo -e "${GREEN}   安装完成！${NC}"
echo -e "${GREEN}================================================${NC}\n"

echo -e "${BLUE}使用方法：${NC}\n"
echo -e "  方法 1: ${YELLOW}直接运行${NC}"
echo -e "    cd $SCRIPT_DIR"
echo -e "    ./pika_launcher.bash\n"

echo -e "  方法 2: ${YELLOW}使用快捷命令（新终端生效）${NC}"
echo -e "    ${GREEN}pika${NC}\n"

echo -e "  方法 3: ${YELLOW}当前终端立即使用${NC}"
echo -e "    source ~/.bashrc"
echo -e "    ${GREEN}pika${NC}\n"

echo -e "${BLUE}推荐: 新开一个终端，直接输入 ${GREEN}pika${NC} 即可启动！\n"

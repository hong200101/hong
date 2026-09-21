# 🚀 Pika 快速开始指南

## 📦 项目文件说明

```
pika_ros-ros2/
├── pika_launcher.bash              # 🎯 交互式一键启动脚本（主程序）
├── setup_launcher.bash             # ⚙️ 安装脚本（一次性运行）
├── PIKA_LAUNCHER_使用说明.md       # 📖 详细使用文档
├── 双Gripper数据采集使用指南.md    # 📘 双Gripper专用指南
└── QUICK_START.md                  # 📋 本文件（快速入门）
```

---

## ⚡ 30 秒快速开始

### 在 Ubuntu 系统上

```bash
# 1. 进入项目目录
cd ~/pika_ros-ros2

# 2. 运行安装脚本（仅首次需要）
bash setup_launcher.bash

# 3. 新开终端，输入快捷命令
pika
```

### 在 Windows/WSL 上

```bash
# 1. 进入项目目录
cd /mnt/c/Users/Hong.Li/Desktop/pika_ros-ros2

# 2. 赋予执行权限
chmod +x pika_launcher.bash

# 3. 运行启动脚本
./pika_launcher.bash
```

---

## 🎯 典型使用场景

### 场景 1️⃣: 首次使用 - 配置双 Gripper

```bash
# 启动 launcher
pika

# 在菜单中选择
1) 配置设备

# 按照提示操作
选择设备类型: 2 (两个 pika gripper)
插入左 Gripper，按回车
拔出左 Gripper，插入右 Gripper，按回车
拔插设备验证绑定

# 完成！
```

### 场景 2️⃣: 启动双 Gripper 采集数据

```bash
# 启动 launcher
pika

# 在菜单中选择
6) 双 Gripper（双机械臂夹爪）

# 是否指定命名空间: n（或输入 y 自定义）

# 系统启动完成后，新开终端记录数据
source ~/pika_ros-ros2/install/setup.bash
ros2 run data_tools data_recorder --output-dir ~/data/episode_001

# 执行机械臂操作任务...

# 停止记录: Ctrl+C
```

### 场景 3️⃣: 处理采集的数据

```bash
# 启动 launcher
pika

# 选择数据同步
12) 数据同步（时间对齐）
  - 数据集目录: ~/data
  - Episode 名称: episode_001
  - 数据类型: gripper
  - 时间差阈值: 0.05

# 选择数据转换
13) 数据转换（生成 HDF5）
  - 数据集目录: ~/data
  - Episode 名称: episode_001
  - 数据类型: gripper
  - 使用索引模式: y
  - 包含点云: n

# 完成！数据保存在 ~/data/episode_001/data.hdf5
```

### 场景 4️⃣: 批量处理多个 episodes

```bash
# 启动 launcher
pika

# 选择批量处理
15) 批量处理（同步+转换）
  - 数据集目录: ~/data
  - 数据类型: gripper

# 自动处理所有 episodes，喝杯咖啡等待 ☕
```

---

## 📚 文档导航

### 🆕 新手必读
1. **[QUICK_START.md](./QUICK_START.md)** ⬅️ 你在这里
2. **[PIKA_LAUNCHER_使用说明.md](./PIKA_LAUNCHER_使用说明.md)** - 详细功能说明

### 📖 深入学习
- **[双Gripper数据采集使用指南.md](./双Gripper数据采集使用指南.md)** - 双 Gripper 完整教程
- **[README_CN.md](./README_CN.md)** - 项目介绍
- **[官方文档](https://agilexsupport.yuque.com/staff-hso6mo/peoot3/vm7e26ho2hmuw0ng)** - Pika 产品手册

---

## 🎨 菜单功能速查

| 分类 | 选项 | 功能 | 常用度 |
|------|------|------|--------|
| 🔧 配置 | 1 | 配置设备 | ⭐⭐⭐⭐⭐ |
| 🔧 配置 | 2 | 检查设备状态 | ⭐⭐⭐⭐ |
| 📹 采集 | 3-9 | 各种采集模式 | ⭐⭐⭐⭐⭐ |
| 🎮 遥操作 | 10-11 | 遥操作模式 | ⭐⭐⭐ |
| 💾 处理 | 12 | 数据同步 | ⭐⭐⭐⭐⭐ |
| 💾 处理 | 13 | 数据转换 | ⭐⭐⭐⭐⭐ |
| 💾 处理 | 14 | 数据加载示例 | ⭐⭐⭐ |
| 💾 处理 | 15 | 批量处理 | ⭐⭐⭐⭐ |
| 🛠️ 工具 | 16-19 | 辅助工具 | ⭐⭐ |

---

## 🔥 常用命令组合

### 完整数据采集流程
```bash
# 终端 1: 启动传感器
pika  # 选择 6) 双 Gripper

# 终端 2: 记录数据
cd ~/pika_ros-ros2
source install/setup.bash
ros2 run data_tools data_recorder --output-dir ~/data/episode_001

# 执行任务后停止记录 (Ctrl+C)

# 终端 1: 处理数据
# 回到 pika launcher
# 选择 12) 数据同步
# 选择 13) 数据转换
```

### 快速验证数据
```bash
pika
# 选择 14) 数据加载示例
# 输入数据集目录: ~/data
# 查看输出验证数据正确性
```

### 多机器人并行采集
```bash
# 终端 1: 机器人 1
pika
# 选择 6) 双 Gripper
# 命名空间: robot1

# 终端 2: 机器人 2
pika
# 选择 6) 双 Gripper
# 命名空间: robot2

# Topics 自动隔离:
# /robot1/gripper/camera_l/...
# /robot2/gripper/camera_l/...
```

---

## ❓ 常见问题

### Q1: 找不到 pika 命令？
```bash
# 解决方案 1: 重新加载配置
source ~/.bashrc
pika

# 解决方案 2: 直接运行
cd ~/pika_ros-ros2
./pika_launcher.bash
```

### Q2: 设备未检测到？
```bash
# 检查 USB 连接
lsusb
ls /dev/ttyUSB*
ls /dev/video*

# 在 launcher 中运行
pika
# 选择 2) 检查设备状态
```

### Q3: ROS2 环境未加载？
```bash
# 手动加载
source /opt/ros/humble/setup.bash
source ~/pika_ros-ros2/install/setup.bash
pika
```

### Q4: 权限被拒绝？
```bash
# 添加到 dialout 组
sudo usermod -a -G dialout $USER
# 重新登录生效

# 或临时赋权
sudo chmod 666 /dev/ttyUSB*
sudo chmod 666 /dev/video*
```

### Q5: 工作空间未构建？
```bash
cd ~/pika_ros-ros2
colcon build --symlink-install
source install/setup.bash
pika
```

---

## 🎓 学习路径

### Level 1: 入门（第 1 天）
- [ ] 运行 `setup_launcher.bash`
- [ ] 使用 `pika` 命令启动
- [ ] 完成设备配置（选项 1）
- [ ] 检查设备状态（选项 2）

### Level 2: 基础采集（第 2-3 天）
- [ ] 启动单个设备（选项 3 或 4）
- [ ] 了解 ROS2 topics
- [ ] 使用 RViz 可视化（选项 16）
- [ ] 采集第一个 episode

### Level 3: 数据处理（第 4-5 天）
- [ ] 运行数据同步（选项 12）
- [ ] 转换为 HDF5（选项 13）
- [ ] 查看数据加载示例（选项 14）
- [ ] 编写自己的 DataLoader

### Level 4: 高级应用（第 6+ 天）
- [ ] 双设备同步采集（选项 5 或 6）
- [ ] 遥操作模式（选项 10 或 11）
- [ ] 批量处理数据（选项 15）
- [ ] 多机器人协同

---

## 🔗 资源链接

### 官方资源
- 🌐 [GitHub 主仓库](https://github.com/agilexrobotics/pika_ros)
- 📦 [Pika SDK](https://github.com/agilexrobotics/pika_sdk)
- 🎮 [PikaAnyArm 遥操作](https://github.com/agilexrobotics/PikaAnyArm)
- 📚 [产品文档](https://agilexsupport.yuque.com/staff-hso6mo/peoot3)

### 技术支持
- 💬 [GitHub Issues](https://github.com/agilexrobotics/pika_ros/issues)
- 📧 Email: support@agilex.ai
- 💡 [QA 问答](https://agilexsupport.yuque.com/staff-hso6mo/peoot3/ltl2m8a3crra12kg)

---

## 🎉 开始你的 Pika 之旅！

```bash
# 就这么简单！
pika
```

**祝数据采集愉快！** 🚀

---

<div align="center">
  <p><strong>Agilex Robotics</strong></p>
  <p>具身智能 · 数据采集解决方案</p>
</div>

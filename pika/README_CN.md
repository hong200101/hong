<div align="center">
  <h1 align="center"> pika_ros </h1>
  <h3 align="center"> Agilex Robotics </h3>
  <p align="center">
    <a href="README.md">English</a> | <a href="README_CN.md">中文</a>
  </p>
</div>
<div align="center">

![ubuntu](https://img.shields.io/badge/Ubuntu-22.04-orange.svg)
![ROS2](https://img.shields.io/badge/ROS-humble-blue.svg)

</div>

## 介绍

Pika 数据套装产品（以下简称Pika）是一款针对**具身智能**领域数据采集场景的**空间数据采集产品**，是一款面向通用操作、轻量化的便携式采执一体化解决方案， 由采集装置及模型推理执行器以及配套的定位基站和数据背包构成。支持高效、准确、快捷、轻量的采集机器人的空间操作数据。

Pika具备超高精度的**毫米级空间信息采集能力**，支持采集数据涵盖六自由度精准空间信息、深度信息、超广角可见光视觉信息以及夹持信息。满足具身智能领域的**数据采集多信息融合需求**。执行器可以基于采集器采集的数据用于模型推理的执行器终端。

## 🚀 快速开始

### ⚡ 30 秒启动

```bash
# 1. 克隆仓库
git clone --recursive https://github.com/agilexrobotics/pika_ros-ros2.git
cd pika_ros-ros2

# 2. 安装 Launcher（一键启动系统）
bash setup_launcher.bash

# 3. 启动！
pika
```

### 📚 文档导航

| 文档 | 说明 | 适合人群 |
|------|------|---------|
| **[QUICK_START.md](./QUICK_START.md)** | 快速开始指南 | 🆕 新手必读 |
| **[PIKA_LAUNCHER_使用说明.md](./PIKA_LAUNCHER_使用说明.md)** | 交互式启动器完整文档 | 📖 详细参考 |
| **[双Gripper数据采集使用指南.md](./双Gripper数据采集使用指南.md)** | 双 Gripper 完整教程 | 🎯 具体场景 |
| **[DEMO_演示脚本.md](./DEMO_演示脚本.md)** | 演示和培训脚本 | 🎬 教学演示 |

### 🎯 典型使用场景

#### 场景 1: 双 Gripper 数据采集
```bash
pika
# 选择: 1) 配置设备 → 6) 双 Gripper → 12) 数据同步 → 13) 转换 HDF5
```

#### 场景 2: 单 Sensor 遥操作
```bash
pika
# 选择: 10) 单 Sensor + 遥操作
```

#### 场景 3: 批量数据处理
```bash
pika
# 选择: 15) 批量处理（自动同步+转换所有 episodes）
```

## 技术支持

如果您在使用过程中遇到任何问题，或者有任何建议和反馈，请通过以下方式联系我们：

- GitHub Issues: https://github.com/agilexrobotics/pika_ros/issues
- 电子邮件: [support@agilex.ai](mailto:support@agilex.ai)

我们的技术团队将尽快回复您的问题，并提供必要的支持和帮助。

### 相关链接

- pika sdk：https://github.com/agilexrobotics/pika_sdk
- pika 遥操作：https://github.com/agilexrobotics/PikaAnyArm
- [Pika 产品用户手册 beta（CN）](https://agilexsupport.yuque.com/staff-hso6mo/peoot3/vm7e26ho2hmuw0ng)
- [PIKA使用QA查询](https://agilexsupport.yuque.com/staff-hso6mo/peoot3/ltl2m8a3crra12kg)

## 支持的环境平台

### 软件环境

- 架构：x86_64
- 操作系统：Ubuntu22.04
- ROS：humble

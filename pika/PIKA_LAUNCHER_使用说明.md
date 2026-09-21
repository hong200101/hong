# 🚀 Pika Launcher - 一键启动系统使用说明

## 📖 简介

`pika_launcher.bash` 是一个交互式的一键启动脚本，集成了 Pika 系统的所有功能：
- ✅ 设备配置与检查
- ✅ 多种采集模式快速启动
- ✅ 遥操作模式
- ✅ 数据处理（同步、转换、加载）
- ✅ 可视化与工具

---

## 🎯 快速开始

### 1. 赋予执行权限

```bash
cd ~/pika_ros-ros2
chmod +x pika_launcher.bash
```

### 2. 启动脚本

```bash
./pika_launcher.bash
```

或直接运行：

```bash
bash pika_launcher.bash
```

---

## 📋 功能菜单详解

### 🔧 设备配置（选项 1-2）

#### **1) 配置设备（首次使用必选）**
- **功能**: 自动检测并绑定 USB 端口
- **使用场景**: 首次使用或更换设备
- **支持模式**:
  - 单个 Sensor/Gripper
  - 双 Sensor/Gripper
  - Sensor + Gripper 混合
  - Helmet（带/不带定位器）
  - 双 Sensor + Helmet

**操作流程**:
```
1. 选择设备组合类型
2. 按提示插拔设备
3. 自动生成配置和启动脚本
4. 验证绑定是否成功
```

#### **2) 检查设备状态**
- **功能**: 诊断硬件连接状态
- **检查项目**:
  - RealSense 深度相机
  - USB 串口设备（ttyUSB*）
  - USB 视频设备（video*）
  - Vive Tracker 定位标签

---

### 📹 数据采集模式（选项 3-9）

#### **3) 单个 Sensor（手持夹爪）**
```bash
设备要求:
- 1 个 Pika Sensor
- 绑定端口: /dev/ttyUSB50, /dev/video50

启动内容:
- RealSense D405 深度相机
- 超广角鱼眼相机
- IMU 9轴传感器
- 夹爪编码器
- Vive 定位系统（如已配置）
```

#### **4) 单个 Gripper（机械臂夹爪）**
```bash
设备要求:
- 1 个 Pika Gripper
- 绑定端口: /dev/ttyUSB50, /dev/video50

启动内容:
- 与单个 Sensor 相同
- 适用于机械臂安装场景
```

#### **5) 双 Sensor（双手持夹爪）**
```bash
设备要求:
- 2 个 Pika Sensor
- 绑定端口: 
  - 左: /dev/ttyUSB50, /dev/video50
  - 右: /dev/ttyUSB51, /dev/video51

启动内容:
- 左右两个完整传感器套件
- 支持自定义命名空间

ROS2 Topics:
/sensor/camera_l/color/image_raw
/sensor/camera_r/color/image_raw
/sensor/gripper_l/data
/sensor/gripper_r/data
/sensor/imu_l/data
/sensor/imu_r/data
```

#### **6) 双 Gripper（双机械臂夹爪）⭐**
```bash
设备要求:
- 2 个 Pika Gripper
- 绑定端口:
  - 左: /dev/ttyUSB60, /dev/video60
  - 右: /dev/ttyUSB61, /dev/video61

启动内容:
- 左右两个 Gripper 传感器套件
- 支持自定义命名空间
- 适合双臂机械臂数据采集

ROS2 Topics:
/gripper/camera_l/color/image_raw
/gripper/camera_r/color/image_raw
/gripper/camera_l/depth/image_rect_raw
/gripper/camera_r/depth/image_rect_raw
/gripper/gripper_l/data
/gripper/gripper_r/data
/gripper/imu_l/data
/gripper/imu_r/data

使用提示:
- 可指定命名空间区分不同机械臂
- 例如: namespace=robot1
```

#### **7) Sensor + Gripper（混合模式）**
```bash
设备要求:
- 1 个 Pika Sensor（手持）
- 1 个 Pika Gripper（机械臂）
- 绑定端口:
  - Sensor: /dev/ttyUSB50
  - Gripper: /dev/ttyUSB60, /dev/video60

使用场景:
- 手持遥操作 + 机械臂执行
- 人机协作数据采集
```

#### **8) Helmet（头盔模式）**
```bash
设备要求:
- 1 个 Pika Helmet
- 绑定端口: /dev/ttyUSB70, /dev/video70

启动内容:
- 头戴式鱼眼相机
- IMU 传感器
- Vive Tracker（可选）

使用场景:
- 第一人称视角采集
- 头部姿态跟踪
```

#### **9) 双 Sensor + Helmet（完整模式）**
```bash
设备要求:
- 2 个 Pika Sensor
- 1 个 Pika Helmet（带 Tracker）
- 绑定端口:
  - Sensor 左: /dev/ttyUSB50, /dev/video50
  - Sensor 右: /dev/ttyUSB51, /dev/video51
  - Helmet: /dev/ttyUSB70, /dev/video70

使用场景:
- 全身动作捕捉
- 多视角数据采集
- 完整具身智能数据集
```

---

### 🎮 遥操作模式（选项 10-11）

#### **10) 单 Sensor + 遥操作**
```bash
功能:
- 启动单个 Sensor
- 启动遥操作控制节点
- 启动 Vive 定位系统

使用场景:
- 手持夹爪遥操作机械臂
- 实时演示和教学
```

#### **11) 双 Sensor + 遥操作**
```bash
功能:
- 启动双 Sensor
- 启动双手遥操作节点
- 双路 Vive 定位

使用场景:
- 双臂机械臂遥操作
- 复杂任务演示
```

---

### 💾 数据处理（选项 12-15）

#### **12) 数据同步（时间对齐）**
```bash
功能: 将不同频率的传感器数据按时间戳对齐

交互参数:
1. 数据集目录 (默认: ~/data)
2. Episode 名称 (留空处理所有)
3. 数据类型 (sensor/gripper/aloha)
4. 时间差阈值 (默认: 0.05s)

输出: 每个数据目录下生成 sync.txt

示例:
数据集目录: ~/data
Episode 名称: episode_001
数据类型: gripper
时间差阈值: 0.05

结果:
~/data/episode_001/camera/color/left/sync.txt
~/data/episode_001/camera/color/right/sync.txt
...
```

#### **13) 数据转换（生成 HDF5）**
```bash
功能: 将同步后的数据转换为 HDF5 训练格式

交互参数:
1. 数据集目录 (默认: ~/data)
2. Episode 名称 (留空处理所有)
3. 数据类型 (sensor/gripper/aloha)
4. 使用索引模式 (y/n, 默认: y)
   - y: 存储文件路径（节省空间）
   - n: 存储实际数据（加载更快）
5. 包含点云数据 (y/n, 默认: n)

输出: ~/data/episode_001/data.hdf5

HDF5 结构:
├── size                          # 数据帧数
├── timestamp                     # 时间戳
├── camera/color/left             # 图像路径或数组
├── camera/depth/left
├── camera/colorIntrinsic/left    # 相机内参
├── gripper/encoderAngle/left     # 夹爪状态
├── imu/9axisOrientation/left     # IMU 数据
└── localization/pose/left        # 6D 位姿
```

#### **14) 数据加载示例**
```bash
功能: 演示如何加载和使用 HDF5 数据

交互参数:
1. 数据集目录 (默认: ~/data)
2. Batch Size (默认: 16)

输出: 打印数据加载示例
```

#### **15) 批量处理（同步+转换）**
```bash
功能: 自动处理目录下所有 episodes

流程:
1. 扫描数据集目录
2. 对每个 episode:
   a. 数据同步
   b. HDF5 转换
3. 输出汇总信息

适用场景: 大批量数据集自动化处理
```

---

### 🛠️ 工具功能（选项 16-19）

#### **16) 启动 RViz 可视化**
```bash
功能: 启动 RViz2 进行数据可视化

可视化内容:
- 相机图像流
- 点云数据
- 机器人模型
- TF 坐标系
- 传感器数据
```

#### **17) 查找 USB 相机**
```bash
功能: 自动扫描并列出所有 USB 相机设备

输出信息:
- 设备路径 (/dev/videoX)
- 制造商 ID (Vendor ID)
- 产品 ID (Product ID)
- 支持的分辨率和帧率
```

#### **18) 相机标定**
```bash
功能: 启动相机标定流程

要求:
- 打印标定板（棋盘格）
- 多角度拍摄标定板
- 计算相机内外参

输出:
- 相机内参矩阵 (K)
- 畸变系数 (D)
- 标定误差报告
```

#### **19) 点云过滤**
```bash
功能: 对点云数据进行预处理

支持操作:
- 降采样（Voxel Grid）
- 离群点去除
- 平面分割
- 聚类

输入: 点云文件路径 (.pcd/.ply)
输出: 过滤后的点云
```

---

## 🎨 界面说明

### 颜色编码
- 🔵 **蓝色 [INFO]**: 提示信息
- 🟢 **绿色 [SUCCESS]**: 操作成功
- 🟡 **黄色 [WARNING]**: 警告信息
- 🔴 **红色 [ERROR]**: 错误信息
- 🟣 **紫色 [STEP]**: 操作步骤

### 菜单分类
- 🟢 **绿色**: 设备配置
- 🔵 **青色**: 数据采集
- 🟡 **黄色**: 遥操作
- 🟣 **紫色**: 数据处理
- 🔵 **蓝色**: 工具功能

---

## 💡 使用技巧

### 1. 首次使用流程
```bash
# 步骤 1: 配置设备
./pika_launcher.bash
选择: 1) 配置设备
选择设备类型: 6 (双 Gripper)
按提示插拔设备

# 步骤 2: 检查设备
选择: 2) 检查设备状态
确认所有设备正常

# 步骤 3: 启动采集
选择: 6) 双 Gripper
确认设备检查通过
开始采集数据
```

### 2. 数据采集流程
```bash
# 启动传感器
选择: 6) 双 Gripper

# 新开终端，启动数据记录
source ~/pika_ros-ros2/install/setup.bash
ros2 run data_tools data_recorder --output-dir ~/data/episode_001

# 执行操作任务
# （控制机械臂进行演示）

# 停止记录 (Ctrl+C)

# 返回 launcher
选择: 12) 数据同步
选择: 13) 数据转换
```

### 3. 批量处理流程
```bash
# 采集多个 episodes
~/data/
├── episode_001/
├── episode_002/
├── episode_003/
└── ...

# 批量处理
选择: 15) 批量处理
输入数据集目录: ~/data
选择数据类型: gripper

# 等待处理完成
# 每个 episode 生成 data.hdf5
```

### 4. 命名空间使用
```bash
# 多机器人场景
选择: 6) 双 Gripper
是否指定命名空间: y
命名空间: robot1

# Topics 变为:
/robot1/gripper/camera_l/color/image_raw
/robot1/gripper/camera_r/color/image_raw
...
```

---

## ⚙️ 环境要求

### 系统要求
- Ubuntu 22.04
- ROS2 Humble
- Python 3.10+

### 依赖检查
脚本会自动检查并加载：
- ROS2 环境 (`/opt/ros/humble/setup.bash`)
- 工作空间环境 (`install/setup.bash`)

### 必需工具
- `rs-enumerate-devices` (librealsense2)
- `v4l2-ctl` (v4l-utils)
- `lsusb` (usbutils)

---

## 🐛 故障排查

### 问题 1: ROS2 环境未加载
```bash
错误: ROS2 环境未加载

解决:
source /opt/ros/humble/setup.bash
source ~/pika_ros-ros2/install/setup.bash
```

### 问题 2: 设备检测失败
```bash
错误: 设备未找到 (ttyUSB50, video50)

解决:
1. 检查 USB 连接
2. 运行: 2) 检查设备状态
3. 重新配置: 1) 配置设备
4. 检查权限: sudo chmod 666 /dev/ttyUSB* /dev/video*
```

### 问题 3: 脚本无执行权限
```bash
错误: Permission denied

解决:
chmod +x pika_launcher.bash
```

### 问题 4: Python 脚本找不到
```bash
错误: xxx.py 不存在

解决:
确保在正确目录: cd ~/pika_ros-ros2
检查 scripts/ 目录是否存在
```

### 问题 5: 工作空间未构建
```bash
警告: 工作空间未构建

解决:
cd ~/pika_ros-ros2
colcon build --symlink-install
source install/setup.bash
```

---

## 📊 典型工作流程

### 场景 1: 双 Gripper 数据采集
```bash
1. 启动 launcher: ./pika_launcher.bash
2. 选择: 6) 双 Gripper
3. 新终端启动数据记录
4. 执行机械臂操作任务
5. 停止记录
6. 选择: 12) 数据同步
7. 选择: 13) 数据转换
8. 选择: 14) 数据加载示例（验证）
```

### 场景 2: 单 Sensor 遥操作
```bash
1. 启动 launcher: ./pika_launcher.bash
2. 选择: 10) 单 Sensor + 遥操作
3. 戴上手持夹爪
4. 控制机械臂执行任务
5. 可选: 同时记录数据
```

### 场景 3: 大批量数据处理
```bash
1. 采集多个 episodes (外部流程)
2. 启动 launcher: ./pika_launcher.bash
3. 选择: 15) 批量处理
4. 输入数据集目录
5. 等待自动处理完成
6. 验证 HDF5 文件
```

---

## 🔗 相关文档

- [双 Gripper 数据采集使用指南](./双Gripper数据采集使用指南.md)
- [Pika 产品用户手册](https://agilexsupport.yuque.com/staff-hso6mo/peoot3/vm7e26ho2hmuw0ng)
- [PIKA 使用 QA](https://agilexsupport.yuque.com/staff-hso6mo/peoot3/ltl2m8a3crra12kg)

---

## 📞 技术支持

- **GitHub Issues**: https://github.com/agilexrobotics/pika_ros/issues
- **邮件**: support@agilex.ai
- **SDK**: https://github.com/agilexrobotics/pika_sdk
- **遥操作**: https://github.com/agilexrobotics/PikaAnyArm

---

## 🎉 更新日志

### v1.0.0 (2024-01-XX)
- ✅ 初始版本发布
- ✅ 支持 9 种采集模式
- ✅ 集成数据处理流程
- ✅ 交互式菜单系统
- ✅ 彩色输出和错误处理
- ✅ 设备自动检测
- ✅ 批量处理功能

---

**祝使用愉快！** 🚀

如有问题，请随时联系技术支持团队。

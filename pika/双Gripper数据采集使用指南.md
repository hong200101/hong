# Pika 双 Gripper 数据采集完整指南

## 📋 目录
1. [环境准备](#1-环境准备)
2. [硬件连接](#2-硬件连接)
3. [设备配置](#3-设备配置)
4. [数据采集](#4-数据采集)
5. [数据同步](#5-数据同步)
6. [数据转换](#6-数据转换)
7. [数据加载训练](#7-数据加载训练)
8. [常见问题](#8-常见问题)

---

## 1. 环境准备

### 1.1 系统要求
- **操作系统**: Ubuntu 22.04
- **ROS2 版本**: Humble
- **架构**: x86_64

### 1.2 安装依赖

#### 安装 RealSense SDK
```bash
# 方法1: 从 source 目录安装（推荐）
cd ~/pika_ros-ros2/source
unzip librealsense-2.55.1.zip
cd librealsense-2.55.1
mkdir build && cd build
cmake ../ -DBUILD_EXAMPLES=true
make -j$(nproc)
sudo make install

# 方法2: 使用 apt 安装
sudo apt-get install ros-humble-librealsense2*
```

#### 克隆 Git 子模块
```bash
cd ~/pika_ros-ros2
git submodule update --init --recursive
```

#### 构建 ROS2 工作空间
```bash
cd ~/pika_ros-ros2
colcon build --symlink-install
source install/setup.bash
```

### 1.3 创建 Python 环境（可选，用于数据处理）
```bash
conda create -n pika_data python=3.10
conda activate pika_data
pip install numpy h5py torch opencv-python pyyaml scipy
```

---

## 2. 硬件连接

### 2.1 设备说明
- **左 Gripper**: 安装在机械臂左侧的夹爪
- **右 Gripper**: 安装在机械臂右侧的夹爪

每个 Gripper 包含：
- ✅ Intel RealSense D405 深度相机（采集 RGB + Depth）
- ✅ 超广角鱼眼相机（180° 视角）
- ✅ IMU 传感器（9轴姿态）
- ✅ 夹爪编码器（夹持状态）

### 2.2 连接步骤
1. **准备两个 USB 3.0 端口**（不同 USB Hub，避免带宽冲突）
2. **插入左 Gripper** 到第一个 USB 口
3. **插入右 Gripper** 到第二个 USB 口
4. **注意**: 配置完成后不能更换 USB 端口！

---

## 3. 设备配置

### 3.1 自动配置（推荐）

```bash
cd ~/pika_ros-ros2/scripts
python3 setup_device.py
```

**按照提示操作：**
1. 输入 `2` 选择"两个 pika gripper"
2. 插入左 Gripper，按回车
3. 拔出左 Gripper，插入右 Gripper，按回车
4. 脚本自动生成以下文件：
   - `setup_multi_gripper.bash` - 设备绑定脚本
   - `start_multi_gripper.bash` - 设备启动脚本

5. 拔插设备测试绑定是否成功

### 3.2 验证绑定

```bash
# 检查串口设备
ls /dev/ttyUSB*
# 应该看到: /dev/ttyUSB60, /dev/ttyUSB61

# 检查视频设备
ls /dev/video*
# 应该看到: /dev/video60, /dev/video61
```

---

## 4. 数据采集

### 4.1 启动传感器节点

```bash
cd ~/pika_ros-ros2/scripts
bash start_multi_gripper.bash
```

**启动的节点包括：**
- ✅ 左右深度相机节点（RealSense D405）
- ✅ 左右鱼眼相机节点（USB Camera）
- ✅ 左右串口节点（IMU + Gripper）

### 4.2 查看 ROS2 话题

```bash
# 新开终端
source ~/pika_ros-ros2/install/setup.bash
ros2 topic list
```

**关键话题列表：**
```
/gripper/camera_l/color/image_raw          # 左深度相机 RGB
/gripper/camera_l/depth/image_rect_raw     # 左深度相机 Depth
/gripper/camera_l/pointcloud               # 左点云
/gripper/camera_fisheye_l/color/image_raw  # 左鱼眼相机

/gripper/camera_r/color/image_raw          # 右深度相机 RGB
/gripper/camera_r/depth/image_rect_raw     # 右深度相机 Depth
/gripper/camera_r/pointcloud               # 右点云
/gripper/camera_fisheye_r/color/image_raw  # 右鱼眼相机

/gripper/gripper_l/data                    # 左夹爪数据
/gripper/gripper_r/data                    # 右夹爪数据
/gripper/imu_l/data                        # 左 IMU 数据
/gripper/imu_r/data                        # 右 IMU 数据
```

### 4.3 启动数据记录

#### 方法1: 使用 data_tools 包（推荐）

```bash
# 新开终端
source ~/pika_ros-ros2/install/setup.bash

# 启动数据记录节点
ros2 run data_tools data_recorder \
  --output-dir ~/data/episode_001 \
  --config-type gripper
```

#### 方法2: 使用 ROS2 bag

```bash
# 记录所有话题
ros2 bag record -a -o ~/data/gripper_data

# 或记录指定话题
ros2 bag record \
  /gripper/camera_l/color/image_raw \
  /gripper/camera_r/color/image_raw \
  /gripper/gripper_l/data \
  /gripper/gripper_r/data \
  -o ~/data/gripper_episode_001
```

### 4.4 执行操作任务

**现在可以进行操作演示：**
- 控制机械臂执行抓取、放置等动作
- 系统自动记录所有传感器数据
- 建议每个任务 episode 持续 10-60 秒

### 4.5 停止记录

```bash
# 按 Ctrl+C 停止数据记录节点
# 数据将保存在指定目录
```

---

## 5. 数据同步

采集的数据需要时间同步，因为不同传感器频率不同。

### 5.1 查看数据目录结构

```bash
cd ~/data/episode_001
tree
```

**预期结构：**
```
episode_001/
├── camera/
│   ├── color/
│   │   ├── left/           # 左相机彩色图
│   │   └── right/          # 右相机彩色图
│   ├── depth/
│   │   ├── left/           # 左相机深度图
│   │   └── right/          # 右相机深度图
│   └── pointCloud/
│       ├── left/           # 左点云
│       └── right/          # 右点云
├── gripper/
│   └── encoder/
│       ├── left/           # 左夹爪编码器
│       └── right/          # 右夹爪编码器
├── imu/
│   └── 9axis/
│       ├── left/           # 左IMU
│       └── right/          # 右IMU
└── localization/
    └── pose/
        ├── left/           # 左定位（如有Tracker）
        └── right/          # 右定位（如有Tracker）
```

### 5.2 执行数据同步

```bash
cd ~/pika_ros-ros2/scripts
python3 data_sync.py \
  --datasetDir ~/data \
  --episodeName episode_001 \
  --type gripper \
  --timeDiffLimit 0.05
```

**参数说明：**
- `--datasetDir`: 数据集根目录
- `--episodeName`: episode 名称
- `--type`: 配置类型（gripper/sensor/aloha）
- `--timeDiffLimit`: 时间差阈值（秒），默认 0.05s

**同步后会生成 `sync.txt` 文件：**
```
episode_001/
├── camera/color/left/sync.txt
├── camera/color/right/sync.txt
├── camera/depth/left/sync.txt
└── ...
```

---

## 6. 数据转换

将同步后的数据转换为 HDF5 格式，方便训练使用。

### 6.1 转换为 HDF5

```bash
cd ~/pika_ros-ros2/scripts
python3 data_to_hdf5.py \
  --datasetDir ~/data \
  --episodeName episode_001 \
  --type gripper \
  --useIndex True \
  --useCameraPointCloud False
```

**参数说明：**
- `--useIndex`: 是否使用索引（True: 存储路径，False: 存储实际数据）
- `--useCameraPointCloud`: 是否包含点云数据（建议 False，节省空间）

**输出文件：**
```
~/data/episode_001/data.hdf5
```

### 6.2 批量转换多个 episode

```bash
# 转换目录下所有 episode
cd ~/pika_ros-ros2/scripts
python3 data_to_hdf5.py \
  --datasetDir ~/data \
  --type gripper \
  --useIndex True
```

### 6.3 HDF5 数据结构

```python
data.hdf5
├── size                                    # 数据帧数
├── timestamp                               # 时间戳数组
├── camera/color/left                       # 图像路径/数组
├── camera/color/right
├── camera/depth/left
├── camera/depth/right
├── camera/colorIntrinsic/left              # 相机内参 (3x3)
├── camera/colorExtrinsic/left              # 相机外参 (4x4)
├── gripper/encoderAngle/left               # 夹爪角度
├── gripper/encoderDistance/left            # 夹爪距离
├── imu/9axisOrientation/left               # IMU 四元数
├── imu/9axisAngularVelocity/left           # 角速度
├── imu/9axisLinearAcceleration/left        # 线性加速度
└── localization/pose/left                  # 6D 位姿 [x,y,z,roll,pitch,yaw]
```

---

## 7. 数据加载训练

### 7.1 使用示例代码加载数据

```bash
cd ~/pika_ros-ros2/scripts
python3 load_data_example.py \
  --datasetDir ~/data \
  --batchSize 16
```

### 7.2 自定义数据加载器

```python
import h5py
import numpy as np
import cv2

# 打开 HDF5 文件
with h5py.File('~/data/episode_001/data.hdf5', 'r') as f:
    # 获取数据大小
    num_frames = f['size'][()]
    
    # 读取某一帧
    frame_idx = 10
    
    # 读取图像（如果 useIndex=True）
    left_color_path = f['camera/color/left'][frame_idx].decode('utf-8')
    left_color_img = cv2.imread(left_color_path)
    
    # 读取夹爪状态
    left_gripper_angle = f['gripper/encoderAngle/left'][frame_idx]
    left_gripper_distance = f['gripper/encoderDistance/left'][frame_idx]
    
    # 读取位姿（如有定位器）
    if 'localization/pose/left' in f:
        left_pose = f['localization/pose/left'][frame_idx]  # [x,y,z,roll,pitch,yaw]
    
    # 读取 IMU
    left_imu_orientation = f['imu/9axisOrientation/left'][frame_idx]  # [x,y,z,w]
    left_imu_angular_vel = f['imu/9axisAngularVelocity/left'][frame_idx]  # [x,y,z]
    left_imu_linear_acc = f['imu/9axisLinearAcceleration/left'][frame_idx]  # [x,y,z]
    
    # 读取相机参数
    left_intrinsic = f['camera/colorIntrinsic/left'][()]  # 3x3 矩阵
    left_extrinsic = f['camera/colorExtrinsic/left'][()]  # 4x4 变换矩阵
```

### 7.3 构建训练数据集

```python
import torch
from torch.utils.data import Dataset, DataLoader

class GripperDataset(Dataset):
    def __init__(self, hdf5_files):
        self.files = hdf5_files
        self.episode_lens = []
        
        for file in hdf5_files:
            with h5py.File(file, 'r') as f:
                self.episode_lens.append(f['size'][()])
        
        self.cumulative_lens = np.cumsum([0] + self.episode_lens)
    
    def __len__(self):
        return self.cumulative_lens[-1]
    
    def __getitem__(self, idx):
        # 定位到对应的 episode 和帧
        episode_idx = np.argmax(self.cumulative_lens > idx) - 1
        frame_idx = idx - self.cumulative_lens[episode_idx]
        
        with h5py.File(self.files[episode_idx], 'r') as f:
            # 读取观测（observation）
            left_color = self._load_image(f['camera/color/left'][frame_idx])
            right_color = self._load_image(f['camera/color/right'][frame_idx])
            left_gripper = f['gripper/encoderAngle/left'][frame_idx]
            right_gripper = f['gripper/encoderAngle/right'][frame_idx]
            
            # 构建状态
            state = np.concatenate([
                [left_gripper],
                [right_gripper]
            ])
            
            # 读取动作（下一帧的状态作为动作）
            if frame_idx < f['size'][()] - 1:
                next_left_gripper = f['gripper/encoderAngle/left'][frame_idx + 1]
                next_right_gripper = f['gripper/encoderAngle/right'][frame_idx + 1]
                action = np.concatenate([
                    [next_left_gripper],
                    [next_right_gripper]
                ])
            else:
                action = state.copy()
            
            return {
                'observation': {
                    'images': {
                        'left': torch.from_numpy(left_color).float(),
                        'right': torch.from_numpy(right_color).float()
                    },
                    'state': torch.from_numpy(state).float()
                },
                'action': torch.from_numpy(action).float()
            }
    
    def _load_image(self, path_bytes):
        path = path_bytes.decode('utf-8')
        img = cv2.imread(path)
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img = cv2.resize(img, (224, 224))
        img = img.transpose(2, 0, 1)  # HWC -> CHW
        img = img / 255.0
        return img

# 使用示例
hdf5_files = [
    '~/data/episode_001/data.hdf5',
    '~/data/episode_002/data.hdf5',
    '~/data/episode_003/data.hdf5'
]

dataset = GripperDataset(hdf5_files)
dataloader = DataLoader(dataset, batch_size=16, shuffle=True, num_workers=4)

for batch in dataloader:
    observations = batch['observation']
    actions = batch['action']
    # 训练你的模型
    # model(observations) -> predicted_actions
    # loss = criterion(predicted_actions, actions)
```

---

## 8. 常见问题

### Q1: 设备未检测到
```bash
# 检查 RealSense 设备
rs-enumerate-devices

# 检查 USB 相机
v4l2-ctl --list-devices

# 检查串口
ls /dev/ttyUSB*
```

### Q2: 权限问题
```bash
# 添加当前用户到 dialout 组
sudo usermod -a -G dialout $USER
# 重新登录生效

# 临时赋予权限
sudo chmod 666 /dev/ttyUSB*
sudo chmod 666 /dev/video*
```

### Q3: USB 带宽不足
- 使用 **不同的 USB 控制器**（不同 USB Hub）
- 降低相机分辨率/帧率：
  ```bash
  camera_fps=15
  camera_width=640
  camera_height=480
  ```

### Q4: 数据同步失败
- 检查 `timeDiffLimit` 参数（增大到 0.1s）
- 确保所有传感器都有数据
- 查看具体错误信息

### Q5: HDF5 文件过大
- 设置 `useIndex=True`（存储路径而非实际图像）
- 不包含点云数据：`useCameraPointCloud=False`
- 压缩图像：JPEG 质量设为 80-90

### Q6: 数据采集频率不一致
不同传感器频率：
- 深度相机：30 Hz
- 鱼眼相机：30 Hz
- IMU：100 Hz
- 夹爪编码器：50 Hz

**解决方案**：使用 `data_sync.py` 时间同步

---

## 🎯 完整工作流程总结

```bash
# 1. 配置设备
cd ~/pika_ros-ros2/scripts
python3 setup_device.py  # 选择 2（双 Gripper）

# 2. 启动传感器
bash start_multi_gripper.bash

# 3. 记录数据（新终端）
source ~/pika_ros-ros2/install/setup.bash
ros2 run data_tools data_recorder --output-dir ~/data/episode_001

# 4. 执行操作任务
# （控制机械臂进行演示）

# 5. 停止记录（Ctrl+C）

# 6. 数据同步
python3 data_sync.py --datasetDir ~/data --episodeName episode_001 --type gripper

# 7. 转换 HDF5
python3 data_to_hdf5.py --datasetDir ~/data --episodeName episode_001 --type gripper

# 8. 加载训练
python3 load_data_example.py --datasetDir ~/data --batchSize 16
```

---

## 📞 技术支持

- **GitHub Issues**: https://github.com/agilexrobotics/pika_ros/issues
- **邮件**: support@agilex.ai
- **文档**: https://agilexsupport.yuque.com/staff-hso6mo/peoot3

祝数据采集顺利！🎉

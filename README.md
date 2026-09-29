## 📦 一、moveit_demos 快速启动脚本

`moveit_demos` 包提供三个快速启动脚本，覆盖从机械臂上电到视觉抓取的完整链路。

| 脚本名称 | 功能说明 | 典型用途 |
|---------|---------|---------|
| **`quick_start`** | 机械臂快速启动脚本，可选机械臂型号与末端工具 | 首次上电、快速验证机械臂与 MoveIt 是否正常 |
| **`moveit_gripper_demo`** | 带夹爪的 MoveIt 规划 Demo | 单独测试夹爪开合与 MoveIt 规划功能 |
| **`start_vision_grasp`** | 抓取蓝色纸巾的快速启动脚本 | 完整视觉抓取流程（识别 → 定位 → 抓取 → 放置） |

### 使用方式

```bash
使用前需要启动机械臂的can通信：
cd agx_arm_ws/src/agx_arm_ros/scripts
bash can_activate.sh

# 1. 机械臂快速启动（可选型号 / 工具）
cd agx_arm_ws/src/agx_arm_ros/moveit_demos
bash quick_start.sh

# 2. 夹爪规划 Demo
ros2 launch moveit_demos moveit_gripper_demo.launch.py

# 3. 蓝色纸巾视觉抓取
cd agx_arm_ws/src/agx_arm_ros/moveit_demos
bash start_vision_grasp.sh



手眼标定流程：
┌─────────────────────────────────────────────────────────────┐
│  Step 1  启动机械臂 + MoveIt                                │
│          
├─────────────────────────────────────────────────────────────┤
│  Step 2  启动相机 + aruco_ros                               │
│    ros2 launch realsense2_camera rs_launch.py               │
│    ros2 launch aruco_ros single.launch.py                   │
├─────────────────────────────────────────────────────────────┤
│  Step 3  启动 easy_handeye2 标定流程                        │
│    ros2 launch easy_handeye2 handeye_calibrate.launch.py \  │
│                                                             │
├─────────────────────────────────────────────────────────────┤
│  Step 4  采样（示教拖动 或 auto_move）                      │
│    采集 15~20 个姿态分散的样本                              │
├─────────────────────────────────────────────────────────────┤
│  Step 5  在 rqt 界面点击 Compute 计算标定矩阵               │
│    查看重投影误差，误差越小越好                              │
├─────────────────────────────────────────────────────────────┤
│  Step 6  点击 Save 保存标定结果到 YAML                      │
├─────────────────────────────────────────────────────────────┤


piper眼在手上标定参数：
Translation
	x: -0.070701
	y: -0.000912
	z: 0.037813
Rotation
	x: -0.133285
	y: 0.126056
	z: -0.659821
	w: 0.728684














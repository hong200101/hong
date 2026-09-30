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


## 📦 二、手眼标定流程

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


## 📦 三、PD + 重力补偿（PD+G） 控制快速启动
    通过 MIT 模式向机械臂发送力矩指令，实现柔顺的阻抗控制效果，可用于：
    推动回位（阻抗回位）柔顺拖动示教  重力补偿悬浮   双边遥操作（配合 remote_force_feedforward）外力估计（配合 external_torque_estimation）

### 使用方式
launch启动，使用前须先启动机械臂
	ros2 launch agx_arm_pd_g_controller pd_g_controller.launch.py \
	  params_file:=$HOME/piper_config/my_pd_g.yaml \   ####替换成具体配置文件的路径
	  enable_gravity_compensation:=true \
	  robot_model:=piper \                             
	  use_gripper:=true

控制器在收到第一条目标参考前不会发布 MIT 指令（安全机制）。必须先发一条目标激活。

	ros2 topic pub --once /control/move_mit_joint_states sensor_msgs/msg/JointState \
	"{name: ['joint1','joint2','joint3','joint4','joint5','joint6'], 
	  position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"

验证是否运行：ros2 topic info /control/move_mit

输出如下：
Publisher count: 1     ← PD+G 控制器
Subscription count: 1  ← 官方驱动


推荐比例参数：kp : kd ≈ 10:1
参考比较柔顺不抖动的值:
    gains:
      joint1: {kp: 1.0, kd: 0.1}
      joint2: {kp: 1.0, kd: 0.1}
      joint3: {kp: 1.0, kd: 0.1}
      joint4: {kp: 0.5, kd: 0.05}
      joint5: {kp: 0.5, kd: 0.05}
      joint6: {kp: 0.5, kd: 0.05}

Piper 固件对 MIT 力矩指令有内部放大，需按版本调整：固件大于1.8使用 torque_scaling: [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]



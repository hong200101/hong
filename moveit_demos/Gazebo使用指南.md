# MoveIt + Gazebo 仿真控制指南

## 📖 简介

本指南介绍如何在 Gazebo 仿真环境中使用 MoveIt 控制机械臂。Gazebo 提供了一个完整的物理仿真环境，让你可以在没有真实机械臂的情况下测试和开发控制程序。

---

## 🎯 Gazebo 仿真的优势

### ✅ 为什么使用 Gazebo？

1. **安全性高**
   - 无需担心损坏真实硬件
   - 可以测试危险或极限动作
   - 不需要真实机械臂就能开发

2. **便利性强**
   - 随时随地开发测试
   - 不受硬件可用性限制
   - 可以快速迭代测试

3. **功能完整**
   - 物理引擎模拟真实动力学
   - 支持碰撞检测
   - 可视化效果好
   - 支持传感器仿真

4. **成本低**
   - 无需购买昂贵硬件
   - 降低开发成本
   - 适合学习和原型开发

---

## 🚀 快速开始

### 方式一：使用 MoveIt Demo Launch（推荐）

这是最简单的方式，MoveIt 会自动配置 Gazebo：

```bash
# 1. 启动 MoveIt + Gazebo 仿真
ros2 launch agx_arm_moveit demo.launch.py \
  arm_type:=piper \
  use_sim_time:=true

# 2. 新终端运行 Demo
cd ~/agx_arm_ws/src/agx_arm_ros/moveit_demos
python3 moveit_gazebo_demo.py
```

### 方式二：手动分步启动

如果你想更多控制，可以分步启动：

```bash
# 终端 1: 启动 Gazebo
gazebo --verbose

# 终端 2: 启动机械臂控制器
ros2 launch agx_arm_description display.launch.py \
  arm_type:=piper \
  use_sim:=true

# 终端 3: 启动 MoveIt
ros2 launch agx_arm_moveit demo.launch.py \
  arm_type:=piper \
  use_sim_time:=true

# 终端 4: 运行 Demo
cd ~/agx_arm_ws/src/agx_arm_ros/moveit_demos
python3 moveit_gazebo_demo.py
```

---

## 📋 Demo 功能说明

### moveit_gazebo_demo.py 包含的功能

#### 1. 基础移动 ⭐⭐⭐⭐⭐
- 关节空间控制
- 笛卡尔空间控制
- 在 Gazebo 中实时可视化

#### 2. 画圆形 ⭐⭐⭐⭐
- 笛卡尔路径规划
- 在 Gazebo 中看到平滑的圆形运动
- 可调整圆心位置和半径

#### 3. 画正方形 ⭐⭐⭐⭐
- 直线段组合
- 在 Gazebo 中看到精确的方形轨迹

#### 4. 抓取和放置 ⭐⭐⭐⭐⭐
- 完整的任务序列
- 包含抓取、搬运、放置
- 模拟夹爪动作

#### 5. 避障演示 ⭐⭐⭐⭐⭐
- 添加虚拟障碍物
- MoveIt 自动规划避障路径
- 在 Gazebo 中可视化障碍物

#### 6. 场景管理 ⭐⭐⭐
- 添加桌面、障碍物
- 清除场景对象
- 实时更新 Gazebo 场景

---

## 🎮 详细操作说明

### 步骤 1: 安装 Gazebo（如果尚未安装）

```bash
# ROS2 Humble
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control

# ROS2 Jazzy
sudo apt install ros-jazzy-gazebo-ros-pkgs ros-jazzy-gazebo-ros2-control

# 验证安装
gazebo --version
```

### 步骤 2: 检查 URDF 配置

确保你的 URDF 文件包含 Gazebo 插件：

```xml
<!-- Gazebo 插件应该在 URDF 中 -->
<gazebo>
  <plugin name="gazebo_ros2_control" filename="libgazebo_ros2_control.so">
    <parameters>$(find agx_arm_moveit)/config/ros2_controllers.yaml</parameters>
  </plugin>
</gazebo>
```

### 步骤 3: 启动仿真

**推荐方式（一键启动）：**

```bash
ros2 launch agx_arm_moveit demo.launch.py \
  arm_type:=piper \
  use_sim_time:=true
```

这会自动启动：
- Gazebo 仿真环境
- 机械臂模型
- MoveIt 规划器
- RViz 可视化

### 步骤 4: 运行 Gazebo Demo

```bash
cd ~/agx_arm_ws/src/agx_arm_ros/moveit_demos
chmod +x moveit_gazebo_demo.py
python3 moveit_gazebo_demo.py
```

---

## 🎓 使用示例

### 示例 1: 基础移动

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import moveit_commander
import sys

rclpy.init()
moveit_commander.roscpp_initialize(sys.argv)

# 创建规划组
arm_group = moveit_commander.MoveGroupCommander('arm')

# 回到零位
arm_group.set_named_target('home')
arm_group.go(wait=True)

# 移动到指定关节角度
arm_group.set_joint_value_target([0.0, 0.4, -0.6, 0.0, 0.0, 0.0])
arm_group.go(wait=True)

# 清理
moveit_commander.roscpp_shutdown()
rclpy.shutdown()
```

### 示例 2: 添加障碍物并避障

```python
from moveit_commander import PlanningSceneInterface
from geometry_msgs.msg import Pose

# 创建场景接口
scene = PlanningSceneInterface()

# 添加桌面
table_pose = Pose()
table_pose.position.z = -0.05
table_pose.orientation.w = 1.0
scene.add_box('table', table_pose, size=(1.0, 1.0, 0.1))

# 添加障碍物
obstacle_pose = Pose()
obstacle_pose.position.x = 0.3
obstacle_pose.position.y = 0.2
obstacle_pose.position.z = 0.3
obstacle_pose.orientation.w = 1.0
scene.add_box('obstacle', obstacle_pose, size=(0.1, 0.1, 0.3))

# 现在规划会自动避开障碍物
arm_group.set_pose_target(target_pose)
arm_group.go(wait=True)

# 清除障碍物
scene.remove_world_object('obstacle')
```

### 示例 3: 在 Gazebo 中画圆

```python
from geometry_msgs.msg import Pose
import math

# 获取当前位姿
current_pose = arm_group.get_current_pose().pose

# 定义圆形参数
center_y = 0.0
center_z = 0.3
radius = 0.05
num_points = 30

# 创建圆形路径点
waypoints = []
for i in range(num_points + 1):
    angle = 2 * math.pi * i / num_points
    
    pose = Pose()
    pose.position.x = current_pose.position.x
    pose.position.y = center_y + radius * math.cos(angle)
    pose.position.z = center_z + radius * math.sin(angle)
    pose.orientation = current_pose.orientation
    
    waypoints.append(pose)

# 规划并执行
(plan, fraction) = arm_group.compute_cartesian_path(waypoints, 0.005, 0.0)
if fraction > 0.95:
    arm_group.execute(plan, wait=True)
```

---

## ⚙️ Gazebo 配置参数

### 重要的 Launch 参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `use_sim_time` | false | 启用 Gazebo 仿真时间 |
| `use_sim` | false | 使用仿真模式 |
| `gui` | true | 启动 Gazebo GUI |
| `paused` | false | Gazebo 启动时暂停 |
| `world` | empty.world | 加载的世界文件 |

### 控制器配置

Gazebo 使用 `ros2_control` 框架，需要配置控制器：

```yaml
# ros2_controllers.yaml
controller_manager:
  ros__parameters:
    update_rate: 100  # Hz
    
    arm_controller:
      type: joint_trajectory_controller/JointTrajectoryController
      
    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

arm_controller:
  ros__parameters:
    joints:
      - joint1
      - joint2
      - joint3
      - joint4
      - joint5
      - joint6
    
    command_interfaces:
      - position
    
    state_interfaces:
      - position
      - velocity
```

---

## 🔧 常见问题解决

### Q1: Gazebo 启动很慢或卡住

**解决方案：**

```bash
# 1. 更新 Gazebo 模型数据库
gazebo --verbose  # 首次运行会下载模型，需要等待

# 2. 关闭不必要的插件
export GAZEBO_MODEL_PATH=$HOME/.gazebo/models

# 3. 使用轻量级世界文件
# 在 launch 文件中指定简单的 world
```

### Q2: 机械臂在 Gazebo 中掉落或抖动

**原因：** 控制器未正确加载

**解决方案：**

```bash
# 1. 检查控制器状态
ros2 control list_controllers

# 2. 启动控制器
ros2 control load_controller arm_controller
ros2 control set_controller_state arm_controller start

# 3. 检查 URDF 中的 Gazebo 插件配置
```

### Q3: MoveIt 规划成功但 Gazebo 中不动

**原因：** 控制接口不匹配

**解决方案：**

```bash
# 1. 检查话题连接
ros2 topic list | grep joint

# 2. 确认控制器话题
ros2 topic echo /arm_controller/joint_trajectory

# 3. 检查 MoveIt 配置中的控制器名称是否正确
```

### Q4: 添加的障碍物在 Gazebo 中看不到

**原因：** 场景同步问题

**解决方案：**

```python
import time

# 添加对象后等待
scene.add_box('obstacle', pose, size=(0.1, 0.1, 0.3))
time.sleep(1.0)  # 等待场景更新

# 或检查对象是否成功添加
known_objects = scene.get_known_object_names()
print(f"场景中的对象: {known_objects}")
```

### Q5: 仿真时间不同步

**解决方案：**

```bash
# 启动时设置 use_sim_time
ros2 launch agx_arm_moveit demo.launch.py use_sim_time:=true

# 或在 Python 代码中
node.declare_parameter('use_sim_time', True)
```

---

## 📊 Gazebo vs 真机对比

| 特性 | Gazebo 仿真 | 真实机械臂 |
|------|-------------|-----------|
| **安全性** | ⭐⭐⭐⭐⭐ | ⭐⭐⭐ |
| **成本** | ⭐⭐⭐⭐⭐ (免费) | ⭐⭐ (需硬件) |
| **可用性** | ⭐⭐⭐⭐⭐ (随时) | ⭐⭐⭐ (受限) |
| **真实性** | ⭐⭐⭐ | ⭐⭐⭐⭐⭐ |
| **延迟** | ⭐⭐⭐⭐ (低) | ⭐⭐⭐⭐⭐ (最低) |
| **精度** | ⭐⭐⭐⭐ | ⭐⭐⭐⭐⭐ |
| **调试** | ⭐⭐⭐⭐⭐ (容易) | ⭐⭐⭐ |

---

## 💡 最佳实践

### 1. 开发流程建议

```
Gazebo 仿真测试 → RViz 可视化验证 → 真机小范围测试 → 真机完整测试
```

### 2. 使用 Gazebo 的场景

**✅ 适合用 Gazebo：**
- 算法原型开发
- 路径规划测试
- 避障算法验证
- 学习和教学
- 危险动作预演

**⚠️ 需要真机：**
- 力控制精细调整
- 视觉伺服最终验证
- 高精度任务
- 生产环境部署

### 3. 性能优化

```bash
# 1. 降低 Gazebo 物理更新频率（提高性能）
export GAZEBO_REAL_TIME_FACTOR=0.5

# 2. 禁用不必要的传感器
# 在 URDF 中注释掉相机、激光雷达等

# 3. 使用无 GUI 模式（服务器上）
ros2 launch agx_arm_moveit demo.launch.py gui:=false
```

### 4. 调试技巧

```bash
# 1. 查看 Gazebo 日志
gazebo --verbose

# 2. 监控控制器状态
ros2 topic echo /arm_controller/state

# 3. 查看 TF 树
ros2 run tf2_tools view_frames

# 4. 录制 bag 文件以便回放
ros2 bag record -a
```

---

## 🎯 学习路径

### 第 1 天：Gazebo 基础
- [ ] 安装和配置 Gazebo
- [ ] 启动基本仿真环境
- [ ] 理解 Gazebo 界面
- [ ] 运行第一个 demo

### 第 2 天：MoveIt + Gazebo
- [ ] 在 Gazebo 中控制机械臂
- [ ] 测试基本运动
- [ ] 理解 ros2_control
- [ ] 运行所有 demo 功能

### 第 3 天：场景和避障
- [ ] 添加障碍物到场景
- [ ] 测试避障规划
- [ ] 创建自定义世界文件
- [ ] 调试场景同步

### 第 4 天：高级功能
- [ ] 传感器仿真
- [ ] 物理参数调整
- [ ] 性能优化
- [ ] 录制和回放

### 第 5 天：实战项目
- [ ] 设计完整任务
- [ ] Gazebo 中完整测试
- [ ] 准备真机部署
- [ ] 文档记录

---

## 📚 相关资源

### 官方文档
- [Gazebo 官方教程](http://gazebosim.org/tutorials)
- [ros2_control 文档](https://control.ros.org/)
- [MoveIt + Gazebo 集成](https://moveit.picknik.ai/main/doc/examples/examples.html)

### 视频教程
- Gazebo 基础入门
- MoveIt Gazebo 集成
- ros2_control 使用

### 示例代码
- [Gazebo ROS2 Examples](https://github.com/ros-simulation/gazebo_ros_demos)
- [MoveIt Tutorials](https://github.com/ros-planning/moveit2_tutorials)

---

## 🔗 快速命令参考

```bash
# 启动 Gazebo + MoveIt
ros2 launch agx_arm_moveit demo.launch.py \
  arm_type:=piper use_sim_time:=true

# 运行 Gazebo Demo
python3 moveit_gazebo_demo.py

# 检查控制器
ros2 control list_controllers

# 查看 Gazebo 模型
gazebo --verbose

# 监控关节状态
ros2 topic echo /joint_states

# 查看 TF
ros2 run tf2_tools view_frames

# 录制仿真
ros2 bag record -a -o gazebo_simulation
```

---

## ⚠️ 注意事项

1. **仿真不等于真实**
   - Gazebo 是近似，不是完美模拟
   - 物理参数需要调校
   - 最终必须在真机验证

2. **性能考虑**
   - Gazebo 比较消耗资源
   - 建议至少 4GB RAM
   - 独立显卡会有帮助

3. **时间同步**
   - 始终使用 `use_sim_time:=true`
   - 注意仿真时间和真实时间的区别

4. **调试建议**
   - 出问题先看日志
   - 使用 `--verbose` 模式
   - 逐步排查各组件

---

## 🎉 开始探索！

现在你已经了解了如何在 Gazebo 中使用 MoveIt 控制机械臂！

**推荐起步：**
```bash
# 1. 启动 Gazebo 仿真
ros2 launch agx_arm_moveit demo.launch.py \
  arm_type:=piper use_sim_time:=true

# 2. 运行 Demo
python3 moveit_gazebo_demo.py

# 3. 选择"2. 在 Gazebo 中画圆形"看效果
```

祝你在 Gazebo 仿真中玩得开心！🤖🎮

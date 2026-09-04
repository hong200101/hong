# MoveIt 控制 Demo 集合

这是一套完整的 MoveIt 控制机械臂的示例程序，适用于 AgileX 机械臂（Piper、Nero 等系列）。

## 📋 目录结构

```
moveit_demos/
├── README.md                    # 本文件
├── moveit_gazebo_demo.py        # Gazebo 仿真控制（新增）⭐
├── moveit_simple_demo.py        # 简单控制示例（推荐新手）
├── moveit_basic_demo.py         # 基础控制示例
├── moveit_advanced_demo.py      # 高级控制示例
├── moveit_gripper_demo.py       # 夹爪控制示例
├── Gazebo使用指南.md            # Gazebo 仿真详细教程
└── requirements.txt             # Python 依赖
```

## 🚀 快速开始

### 1. 前置条件

确保已完成以下安装：

- ROS2 (Humble 或 Jazzy)
- AgileX 机械臂驱动包
- MoveIt2
- Python 依赖

### 2. 安装 Python 依赖

```bash
cd ~/agx_arm_ws/src/agx_arm_ros/moveit_demos
pip3 install -r requirements.txt
```

或手动安装：

```bash
# Humble
pip3 install transforms3d

# Jazzy
pip3 install transforms3d --break-system-packages
```

### 3. 启动机械臂和 MoveIt

**方式一：一键启动（推荐）**

```bash
# 仅机械臂（无末端执行器）
ros2 launch agx_arm_ctrl start_single_agx_arm_moveit.launch.py \
  can_port:=can0 arm_type:=piper

# 带夹爪
ros2 launch agx_arm_ctrl start_single_agx_arm_moveit.launch.py \
  can_port:=can0 arm_type:=piper effector_type:=agx_gripper

# 带灵巧手
ros2 launch agx_arm_ctrl start_single_agx_arm_moveit.launch.py \
  can_port:=can0 arm_type:=piper effector_type:=revo2 revo2_type:=left
```

**方式二：分步启动**

终端 1 - 启动机械臂控制：
```bash
ros2 launch agx_arm_ctrl start_single_agx_arm.launch.py \
  can_port:=can0 arm_type:=piper effector_type:=agx_gripper
```

终端 2 - 启动 MoveIt：
```bash
ros2 launch agx_arm_moveit demo.launch.py \
  arm_type:=piper effector_type:=agx_gripper follow:=true
```

### 4. 运行 Demo

在新终端中：

```bash
cd ~/agx_arm_ws
source install/setup.bash
cd src/agx_arm_ros/moveit_demos

# 给脚本添加执行权限
chmod +x *.py

# 运行示例
python3 moveit_simple_demo.py
```

## 📚 Demo 说明

### 0. moveit_gazebo_demo.py - Gazebo 仿真控制 ⭐ 新增

**功能特点：**
- 在 Gazebo 物理仿真环境中运行
- 完整的物理引擎支持
- 可视化避障和场景管理
- 无需真实机械臂

**包含演示：**
- ✅ 基础移动（Gazebo 环境）
- ✅ 在 Gazebo 中画圆形
- ✅ 在 Gazebo 中画正方形
- ✅ Gazebo 抓取和放置演示
- ✅ 避障演示（添加虚拟障碍物）
- ✅ 场景对象管理

**运行方式：**
```bash
# 1. 启动 Gazebo + MoveIt
ros2 launch agx_arm_moveit demo.launch.py \
  arm_type:=piper use_sim_time:=true

# 2. 运行 Demo
python3 moveit_gazebo_demo.py
```

**详细说明：** 参见 `Gazebo使用指南.md`

---

### 1. moveit_simple_demo.py - 简单控制示例 ⭐ 推荐新手

**功能特点：**
- 使用标准 MoveIt Commander API
- 代码简洁易懂
- 交互式菜单操作
- 包含基础和高级功能

**包含演示：**
- ✅ 关节空间运动控制
- ✅ 笛卡尔空间路径规划
- ✅ 画正方形
- ✅ 画圆形
- ✅ 预设位置控制

**运行方式：**
```bash
python3 moveit_simple_demo.py
```

**使用示例：**
```
请选择演示项目:
1. 基础移动 (关节空间)
2. 画正方形 (笛卡尔路径)
3. 画圆形 (笛卡尔路径)
4. 回到零位
0. 退出
```

---

### 2. moveit_basic_demo.py - 基础控制示例

**功能特点：**
- 关节空间控制
- 笛卡尔空间控制
- 逐步执行教学

**适用场景：**
- 学习 MoveIt 基本概念
- 了解关节和笛卡尔空间的区别
- 实验不同的控制方式

**运行方式：**
```bash
python3 moveit_basic_demo.py
```

**主要功能：**
```python
# 回到零位
demo.go_to_home()

# 关节空间控制
demo.go_to_joint_position([0.0, 0.4, -0.6, 0.0, 0.0, 0.0])

# 笛卡尔空间控制
demo.go_to_pose(x=0.3, y=0.0, z=0.3, roll=0.0, pitch=0.0, yaw=0.0)

# 打印当前状态
demo.print_current_state()
```

---

### 3. moveit_advanced_demo.py - 高级控制示例

**功能特点：**
- 复杂轨迹规划
- 笛卡尔路径插值
- 避障功能（如果配置了场景）
- 抓取和放置序列

**适用场景：**
- 高级运动规划
- 复杂任务编排
- 自定义轨迹生成

**运行方式：**
```bash
python3 moveit_advanced_demo.py
```

**包含演示：**
- ✅ 画圆形（参数化）
- ✅ 画正方形
- ✅ 自定义笛卡尔路径
- ✅ 抓取和放置演示

**使用示例：**
```python
# 画圆
plan = demo.draw_circle(
    center_x=0.3,
    center_y=0.0,
    center_z=0.3,
    radius=0.05,
    num_points=30
)
demo.execute_plan(plan)

# 画正方形
plan = demo.draw_square(
    start_x=0.25,
    start_y=-0.05,
    start_z=0.3,
    side_length=0.1
)
demo.execute_plan(plan)

# 自定义路径
waypoints = [
    (0.3, 0.0, 0.2, 0.0, 0.0, 0.0),
    (0.3, 0.1, 0.3, 0.0, 0.0, 0.0),
    (0.3, 0.0, 0.4, 0.0, 0.0, 0.0),
]
plan = demo.plan_cartesian_path(waypoints)
demo.execute_plan(plan)
```

---

### 4. moveit_gripper_demo.py - 夹爪控制示例

**功能特点：**
- 机械臂 + 夹爪联合控制
- 完整抓取和放置序列
- 夹爪力控制
- 夹爪宽度精确控制

**适用场景：**
- 抓取任务
- 物体操作
- 装配任务

**运行前提：**
必须启动带夹爪的配置：
```bash
ros2 launch agx_arm_ctrl start_single_agx_arm_moveit.launch.py \
  can_port:=can0 arm_type:=piper effector_type:=agx_gripper
```

**运行方式：**
```bash
python3 moveit_gripper_demo.py
```

**包含演示：**
- ✅ 夹爪测试序列
- ✅ 完整抓取和放置
- ✅ 手动控制夹爪

**使用示例：**
```python
# 打开夹爪
demo.open_gripper(width=0.08, force=1.0)

# 关闭夹爪
demo.close_gripper(width=0.0, force=1.5)

# 设置特定宽度
demo.set_gripper_width(width=0.05, force=2.0)

# 完整抓取和放置
demo.pick_and_place_sequence()
```

---

## 🎯 API 参考

### 常用函数

#### 1. 机械臂控制

```python
# 回到零位
go_to_home()

# 关节空间控制
go_to_joint_position([j1, j2, j3, j4, j5, j6])
# 参数：关节角度列表（弧度）

# 笛卡尔空间控制
go_to_pose(x, y, z, roll, pitch, yaw)
# 参数：位置（米）+ 姿态（弧度）

# 打印当前状态
print_current_state()
```

#### 2. 笛卡尔路径规划

```python
# 规划笛卡尔路径
plan = plan_cartesian_path(waypoints, eef_step=0.01)
# waypoints: [(x, y, z, r, p, y), ...]
# eef_step: 末端执行器步进大小（米）

# 执行规划
execute_plan(plan)
```

#### 3. 夹爪控制

```python
# 打开夹爪
open_gripper(width=0.1, force=1.0)
# width: 0.0-0.1 米
# force: 0.5-3.0 牛顿

# 关闭夹爪
close_gripper(width=0.0, force=1.5)

# 设置任意宽度
set_gripper_width(width=0.05, force=2.0)
```

### 参数范围

| 参数 | 范围 | 单位 | 说明 |
|------|------|------|------|
| 关节角度 | 依机型而定 | 弧度 | 见机械臂文档 |
| 位置 (x, y, z) | 依机型而定 | 米 | 笛卡尔空间 |
| 姿态 (r, p, y) | [-π, π] | 弧度 | 欧拉角 |
| 夹爪宽度 | [0.0, 0.1] | 米 | 开口距离 |
| 夹爪力 | [0.5, 3.0] | 牛顿 | 夹持力度 |
| 速度比例 | [0.0, 1.0] | - | 最大速度的比例 |
| 加速度比例 | [0.0, 1.0] | - | 最大加速度的比例 |

---

## ⚙️ 配置参数

### MoveIt 规划参数

```python
# 设置速度和加速度比例
arm_group.set_max_velocity_scaling_factor(0.3)      # 30% 最大速度
arm_group.set_max_acceleration_scaling_factor(0.3)  # 30% 最大加速度

# 设置规划时间
arm_group.set_planning_time(10.0)  # 秒

# 设置位姿容差
arm_group.set_goal_position_tolerance(0.01)      # 1cm
arm_group.set_goal_orientation_tolerance(0.05)   # 约3度

# 设置参考坐标系
arm_group.set_pose_reference_frame('base_link')
```

### 笛卡尔路径参数

```python
# eef_step: 末端执行器步进大小
# 值越小，路径越平滑，但计算量越大
eef_step = 0.01  # 1cm

# jump_threshold: 跳跃阈值
# 0.0 表示不允许跳跃
jump_threshold = 0.0
```

---

## 🔧 故障排除

### 问题 1: 导入错误 - 找不到 moveit_commander

**症状：**
```
ModuleNotFoundError: No module named 'moveit_commander'
```

**解决方案：**
```bash
# 确保安装了 MoveIt2
sudo apt install ros-$ROS_DISTRO-moveit*

# 确保已 source 工作空间
source /opt/ros/$ROS_DISTRO/setup.bash
source ~/agx_arm_ws/install/setup.bash
```

### 问题 2: 规划失败

**症状：**
```
✗ 移动失败
✗ 路径规划不完整
```

**可能原因和解决方案：**

1. **目标位置超出工作空间**
   - 检查目标位置是否在机械臂可达范围内
   - 减小移动距离或调整目标位置

2. **速度/加速度设置过高**
   ```python
   arm_group.set_max_velocity_scaling_factor(0.2)  # 降低到 20%
   arm_group.set_max_acceleration_scaling_factor(0.2)
   ```

3. **规划时间不足**
   ```python
   arm_group.set_planning_time(15.0)  # 增加到 15 秒
   ```

4. **逆运动学无解**
   - 调整目标姿态
   - 尝试使用关节空间控制代替笛卡尔空间控制

### 问题 3: 夹爪不工作

**症状：**
```
⚠ 未找到夹爪规划组
```

**解决方案：**

1. 确保启动时指定了夹爪：
   ```bash
   effector_type:=agx_gripper
   ```

2. 检查夹爪话题：
   ```bash
   ros2 topic list | grep gripper
   ros2 topic echo /feedback/gripper_status
   ```

3. 手动测试夹爪：
   ```bash
   ros2 topic pub /control/joint_states sensor_msgs/msg/JointState \
     "{name: [gripper], position: [0.05], effort: [1.0]}" -1
   ```

### 问题 4: 机械臂不移动

**症状：**
- 规划成功但机械臂不动

**解决方案：**

1. 检查机械臂是否使能：
   ```bash
   ros2 service call /enable_agx_arm std_srvs/srv/SetBool "{data: true}"
   ```

2. 检查控制门是否打开（如果使用 auto_control_gate）：
   ```bash
   ros2 service call /control_enable std_srvs/srv/SetBool "{data: true}"
   ```

3. 检查反馈话题：
   ```bash
   ros2 topic echo /feedback/joint_states
   ros2 topic echo /feedback/arm_status
   ```

### 问题 5: LC_NUMERIC 错误

**症状：**
```
参数需要一个 double，而提供的是一个 string
```

**解决方案：**
```bash
echo "export LC_NUMERIC=en_US.UTF-8" >> ~/.bashrc
source ~/.bashrc
```

---

## 📖 进阶使用

### 1. 添加碰撞物体

```python
from geometry_msgs.msg import PoseStamped

# 添加桌面
table_pose = PoseStamped()
table_pose.header.frame_id = "base_link"
table_pose.pose.position.z = -0.05
demo.scene.add_box("table", table_pose, size=(1.0, 1.0, 0.1))

# 添加障碍物
obstacle_pose = PoseStamped()
obstacle_pose.header.frame_id = "base_link"
obstacle_pose.pose.position.x = 0.3
obstacle_pose.pose.position.y = 0.2
obstacle_pose.pose.position.z = 0.3
demo.scene.add_box("obstacle", obstacle_pose, size=(0.1, 0.1, 0.3))
```

### 2. 附加和分离物体

```python
# 附加物体到末端执行器
eef_link = arm_group.get_end_effector_link()
demo.scene.attach_box(eef_link, "object", touch_links=[eef_link])

# 分离物体
demo.scene.remove_attached_object(eef_link, name="object")
```

### 3. 轨迹时间参数化

```python
from moveit_msgs.msg import RobotTrajectory
from trajectory_processing import RobotTrajectoryProcess

# 重新计算轨迹时间
robot_trajectory = RobotTrajectory()
# ... 规划轨迹 ...
# 使用 iterative spline 或 time-optimal 参数化
```

### 4. 自定义约束

```python
from moveit_msgs.msg import Constraints, OrientationConstraint

# 添加姿态约束
constraints = Constraints()
orientation_constraint = OrientationConstraint()
orientation_constraint.header.frame_id = "base_link"
orientation_constraint.link_name = arm_group.get_end_effector_link()
# 设置约束参数...
constraints.orientation_constraints.append(orientation_constraint)

arm_group.set_path_constraints(constraints)
```

---

## 📝 最佳实践

### 1. 运动规划

- ✅ 优先使用关节空间控制，更可靠
- ✅ 笛卡尔路径适用于直线或弧线运动
- ✅ 设置合理的速度和加速度（建议从 0.2-0.3 开始）
- ✅ 给规划器足够的时间（10-15 秒）
- ✅ 始终检查规划返回值

### 2. 安全考虑

- ⚠️ 首次运行时设置低速度
- ⚠️ 保持安全距离
- ⚠️ 准备好急停按钮
- ⚠️ 检查工作空间内无障碍物
- ⚠️ 使用仿真环境测试新代码

### 3. 调试技巧

- 🔍 使用 `print_current_state()` 监控状态
- 🔍 在 RViz 中可视化规划结果
- 🔍 检查日志输出
- 🔍 使用 `ros2 topic echo` 监控话题
- 🔍 降低速度以便观察运动

### 4. 代码组织

- 📦 将重复的动作封装成函数
- 📦 使用配置文件存储位置参数
- 📦 添加错误处理和重试逻辑
- 📦 记录重要的状态变化

---

## 🎓 学习路径

### 初学者
1. ✅ 阅读 `moveit_simple_demo.py`
2. ✅ 运行基础移动演示
3. ✅ 理解关节空间 vs 笛卡尔空间
4. ✅ 尝试修改参数观察变化

### 进阶
1. ✅ 学习 `moveit_advanced_demo.py`
2. ✅ 自定义笛卡尔路径
3. ✅ 添加碰撞检测
4. ✅ 编写自己的抓取序列

### 高级
1. ✅ 学习轨迹优化
2. ✅ 实现复杂的任务规划
3. ✅ 集成视觉系统
4. ✅ 开发生产应用

---

## 📚 相关资源

### 官方文档
- [MoveIt2 官方文档](https://moveit.picknik.ai/main/index.html)
- [AgileX 机械臂文档](../../README.md)
- [ROS2 教程](https://docs.ros.org/en/humble/Tutorials.html)

### 示例视频
- MoveIt 基础教程
- 机械臂运动规划
- 抓取和放置演示

### 社区支持
- ROS Answers
- MoveIt GitHub
- AgileX 用户论坛

---

## 🤝 贡献

欢迎提交问题报告、功能请求或代码改进！

---

## 📄 许可证

本示例代码遵循与主仓库相同的许可证。

---

## ✉️ 联系方式

如有问题或建议，请通过以下方式联系：
- 提交 GitHub Issue
- 发送邮件到技术支持
- 访问官方论坛

---

**祝你使用愉快！Happy Coding! 🎉**

# Gazebo 仿真快速参考卡

## 🚀 一分钟快速启动

```bash
# 方式 1: 使用快速启动脚本（推荐）
cd ~/agx_arm_ws/src/agx_arm_ros/moveit_demos
chmod +x start_gazebo.sh
./start_gazebo.sh

# 方式 2: 手动启动
ros2 launch agx_arm_moveit demo.launch.py \
  arm_type:=piper use_sim_time:=true

# 新终端运行 Demo
python3 moveit_gazebo_demo.py
```

---

## 📋 常用命令

### Gazebo 启动

```bash
# 完整可视化（Gazebo + MoveIt + RViz）
ros2 launch agx_arm_moveit demo.launch.py \
  arm_type:=piper use_sim_time:=true

# 无 RViz（轻量级）
ros2 launch agx_arm_moveit demo.launch.py \
  arm_type:=piper use_sim_time:=true use_rviz:=false

# 无 Gazebo GUI（服务器模式）
ros2 launch agx_arm_moveit demo.launch.py \
  arm_type:=piper use_sim_time:=true gui:=false

# 带夹爪
ros2 launch agx_arm_moveit demo.launch.py \
  arm_type:=piper effector_type:=agx_gripper use_sim_time:=true
```

### Gazebo 控制

```bash
# 暂停/继续仿真
gz world -p 1  # 暂停
gz world -p 0  # 继续

# 查看 Gazebo 版本
gazebo --version

# 清理 Gazebo 缓存
rm -rf ~/.gazebo/models/*
```

### 检查和调试

```bash
# 查看控制器状态
ros2 control list_controllers

# 查看关节状态
ros2 topic echo /joint_states

# 查看 TF 树
ros2 run tf2_tools view_frames

# 监控 Gazebo 话题
gz topic -l
```

---

## 🎮 Gazebo Demo 菜单

运行 `python3 moveit_gazebo_demo.py` 后的菜单：

```
1. 基础移动 (关节空间)      - Gazebo 中的基本运动
2. 在 Gazebo 中画圆形        - 圆形轨迹演示
3. 在 Gazebo 中画正方形      - 方形轨迹演示
4. Gazebo 抓取和放置演示    - 完整任务序列
5. Gazebo 避障演示          - 添加障碍物并规划
6. 添加/清除场景对象        - 场景管理
7. 查看当前状态
8. 回到零位
0. 退出
```

---

## 💻 代码片段

### 基础控制

```python
import rclpy
import moveit_commander
import sys

# 初始化
rclpy.init()
moveit_commander.roscpp_initialize(sys.argv)

# 创建规划组
arm_group = moveit_commander.MoveGroupCommander('arm')

# 回到零位
arm_group.set_named_target('home')
arm_group.go(wait=True)

# 关节控制
arm_group.set_joint_value_target([0.0, 0.4, -0.6, 0.0, 0.0, 0.0])
arm_group.go(wait=True)
```

### 添加障碍物

```python
from moveit_commander import PlanningSceneInterface
from geometry_msgs.msg import Pose

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

# 清除对象
scene.remove_world_object('obstacle')
```

### 笛卡尔路径

```python
from geometry_msgs.msg import Pose
import math

# 圆形路径
waypoints = []
center_y, center_z = 0.0, 0.3
radius = 0.05

for i in range(31):
    angle = 2 * math.pi * i / 30
    pose = Pose()
    pose.position.x = 0.3
    pose.position.y = center_y + radius * math.cos(angle)
    pose.position.z = center_z + radius * math.sin(angle)
    pose.orientation.w = 1.0
    waypoints.append(pose)

# 规划并执行
(plan, fraction) = arm_group.compute_cartesian_path(waypoints, 0.005, 0.0)
if fraction > 0.95:
    arm_group.execute(plan, wait=True)
```

---

## ⚙️ 重要参数

### Launch 参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `use_sim_time` | false | **必须设为 true** |
| `gui` | true | Gazebo GUI |
| `use_rviz` | true | 启动 RViz |
| `paused` | false | 暂停启动 |
| `verbose` | false | 详细输出 |

### Gazebo 环境变量

```bash
# 设置实时系数（提高性能）
export GAZEBO_REAL_TIME_FACTOR=0.5

# 模型路径
export GAZEBO_MODEL_PATH=~/agx_arm_ws/src/agx_arm_ros/models

# 资源路径
export GAZEBO_RESOURCE_PATH=~/agx_arm_ws/src/agx_arm_ros
```

---

## 🆘 快速故障排除

| 问题 | 解决方案 |
|------|----------|
| Gazebo 启动慢 | `gazebo --verbose` 首次下载模型 |
| 机械臂掉落 | 检查控制器：`ros2 control list_controllers` |
| 不执行轨迹 | 确认 `use_sim_time:=true` |
| 场景对象不显示 | 添加后 `time.sleep(1.0)` |
| 内存占用高 | 关闭 RViz：`use_rviz:=false` |

---

## 🔧 性能优化

```bash
# 1. 降低物理更新频率
# 在 world 文件中设置 <max_step_size>0.01</max_step_size>

# 2. 禁用传感器
# 注释 URDF 中的相机、激光雷达

# 3. 使用无 GUI 模式
gui:=false

# 4. 限制 Gazebo 资源
export GAZEBO_REAL_TIME_FACTOR=0.5
```

---

## 📊 Gazebo vs 真机

| 特性 | Gazebo | 真机 |
|------|--------|------|
| 安全性 | ⭐⭐⭐⭐⭐ | ⭐⭐⭐ |
| 成本 | 免费 | 需硬件 |
| 可用性 | 随时 | 受限 |
| 真实性 | ⭐⭐⭐ | ⭐⭐⭐⭐⭐ |
| 精度 | ⭐⭐⭐⭐ | ⭐⭐⭐⭐⭐ |
| 调试难度 | 容易 | 中等 |

---

## 💡 使用技巧

### 1. 先仿真后真机
```
Gazebo 测试 → RViz 验证 → 真机小范围测试 → 真机完整测试
```

### 2. 录制仿真数据
```bash
ros2 bag record -a -o gazebo_test
ros2 bag play gazebo_test
```

### 3. 调试物理参数
```xml
<!-- URDF 中调整 -->
<gazebo reference="link_name">
  <mu1>0.8</mu1>
  <mu2>0.8</mu2>
  <kp>1000000.0</kp>
  <kd>1.0</kd>
</gazebo>
```

### 4. 使用 Gazebo 插件
```xml
<gazebo>
  <plugin name="gazebo_ros2_control" filename="libgazebo_ros2_control.so">
    <parameters>config/ros2_controllers.yaml</parameters>
  </plugin>
</gazebo>
```

---

## 🎓 学习路径

**第 1 天：** Gazebo 基础
- [ ] 安装和启动 Gazebo
- [ ] 运行第一个 demo
- [ ] 理解 Gazebo 界面

**第 2 天：** MoveIt 集成
- [ ] 在 Gazebo 中控制机械臂
- [ ] 测试所有 demo 功能
- [ ] 理解 ros2_control

**第 3 天：** 场景和避障
- [ ] 添加障碍物
- [ ] 测试避障规划
- [ ] 创建自定义场景

**第 4 天：** 高级功能
- [ ] 调整物理参数
- [ ] 性能优化
- [ ] 录制和回放

**第 5 天：** 实战项目
- [ ] 设计完整任务
- [ ] Gazebo 中测试
- [ ] 准备真机部署

---

## 📚 相关资源

- [Gazebo 官方教程](http://gazebosim.org/tutorials)
- [ros2_control 文档](https://control.ros.org/)
- [详细教程](./Gazebo使用指南.md)

---

## 🎉 快速开始

```bash
# 1. 启动 Gazebo
./start_gazebo.sh

# 或手动启动
ros2 launch agx_arm_moveit demo.launch.py \
  arm_type:=piper use_sim_time:=true

# 2. 新终端运行 Demo
python3 moveit_gazebo_demo.py

# 3. 选择功能测试
```

**祝你在 Gazebo 仿真中玩得开心！🤖🎮**

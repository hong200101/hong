#!/usr/bin/env python3
"""
MoveIt 机械臂 + AGX Gripper 夹爪控制 Demo - 完整集成版
适配 Piper 机械臂和 AGX Gripper
"""

import sys
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup

# ROS2 消息类型
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest,
    Constraints,
    JointConstraint,
    PositionConstraint,
    BoundingVolume,
    PlanningOptions,
    RobotState,
)
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose, Quaternion
from std_msgs.msg import Header
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory

# 使用 transforms3d 进行四元数转换
from transforms3d.euler import euler2quat

import time
import math


class MoveItGripperDemo(Node):
    def __init__(self):
        super().__init__('moveit_gripper_demo')
        
        self.get_logger().info('='*60)
        self.get_logger().info('初始化 Piper 机械臂 + AGX Gripper 控制 Demo...')
        self.get_logger().info('='*60)
        
        # ========== 机械臂部分 ==========
        # 订阅关节状态
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/control/joint_states',  # 使用实际发布的关节状态话题
            self.joint_state_callback,
            10
        )
        
        self.current_joint_names = []
        self.current_joint_positions = []
        
        # 创建机械臂 MoveGroup action 客户端
        self.arm_move_group_client = ActionClient(
            self,
            MoveGroup,
            '/move_action',
            callback_group=ReentrantCallbackGroup()
        )
        
        if not self.arm_move_group_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('✗ 无法连接到机械臂 MoveGroup action 服务器')
            raise RuntimeError("机械臂 MoveGroup 不可用")
        
        self.get_logger().info('✓ 成功连接到机械臂 MoveGroup action 服务器')
        
        # Piper 机械臂配置
        self.planning_group = 'arm'
        self.robot_base_frame = 'base_link'
        self.robot_tip_frame = 'link6'
        
        self.arm_joint_names = [
            'joint1', 'joint2', 'joint3',
            'joint4', 'joint5', 'joint6'
        ]
        
        # 关节限制（根据 URDF）
        self.joint_limits = [
            (-2.6179938, 2.6179938),    # joint1
            (0.0, 3.1415926),            # joint2
            (-2.9670597, 0.0),           # joint3
            (-1.7453292, 1.7453292),     # joint4
            (-1.2217304, 1.2217304),     # joint5
            (-2.0943951, 2.0943951),     # joint6
        ]
        
        # ========== 夹爪部分 ==========
        # 订阅夹爪状态
        self.gripper_status_sub = self.create_subscription(
            JointState,
            '/feedback/gripper_status',
            self.gripper_status_callback,
            10
        )
        
        # 创建夹爪 joint_trajectory 发布者
        self.gripper_trajectory_pub = self.create_publisher(
            JointTrajectory,
            '/gripper_controller/joint_trajectory',
            10
        )
        
        # 创建控制话题发布者（备用）
        self.control_joint_pub = self.create_publisher(
            JointState,
            '/control/joint_states',
            10
        )
        
        # 尝试创建夹爪 action 客户端
        self.gripper_action_client = None
        try:
            self.gripper_action_client = ActionClient(
                self,
                FollowJointTrajectory,
                '/gripper_controller/follow_joint_trajectory',
                callback_group=ReentrantCallbackGroup()
            )
            if self.gripper_action_client.wait_for_server(timeout_sec=2.0):
                self.get_logger().info('✓ 找到夹爪 FollowJointTrajectory action')
            else:
                self.gripper_action_client = None
        except:
            pass
        
        # 夹爪状态
        self.current_gripper_position = None
        self.gripper_joint_name = 'gripper'
        self.gripper_force = 1.0
        
        self.get_logger().info('✓ 初始化完成')
        self.get_logger().info(f'  机械臂规划组: {self.planning_group}')
        self.get_logger().info(f'  末端执行器: {self.robot_tip_frame}')
        self.get_logger().info(f'  夹爪关节: {self.gripper_joint_name}')
        if self.gripper_action_client:
            self.get_logger().info('  夹爪控制: Action + Trajectory')
        else:
            self.get_logger().info('  夹爪控制: Trajectory')
    
    # ========== 回调函数 ==========
    def joint_state_callback(self, msg):
        """关节状态回调"""
        self.current_joint_names = list(msg.name)
        self.current_joint_positions = list(msg.position)
    
    def gripper_status_callback(self, msg):
        """夹爪状态回调"""
        if msg.name and msg.position:
            self.gripper_joint_name = msg.name[0] if msg.name else 'gripper'
            self.current_gripper_position = msg.position[0]
    
    # ========== 机械臂控制 ==========
    def get_current_joint_positions(self):
        """获取当前关节位置"""
        if self.current_joint_positions:
            return self.current_joint_positions
        return None
    
    def create_joint_constraints(self, joint_names, joint_values):
        """创建关节约束"""
        constraints = Constraints()
        
        for i, (name, value) in enumerate(zip(joint_names, joint_values)):
            joint_constraint = JointConstraint()
            joint_constraint.joint_name = name
            joint_constraint.position = value
            joint_constraint.tolerance_above = 0.01
            joint_constraint.tolerance_below = 0.01
            joint_constraint.weight = 1.0
            constraints.joint_constraints.append(joint_constraint)
        
        return constraints
    
    def create_position_constraints(self, x, y, z):
        """创建位置约束"""
        constraints = Constraints()
        
        position_constraint = PositionConstraint()
        position_constraint.header = Header()
        position_constraint.header.frame_id = self.robot_base_frame
        position_constraint.link_name = self.robot_tip_frame
        
        position_constraint.target_point_offset.x = x
        position_constraint.target_point_offset.y = y
        position_constraint.target_point_offset.z = z
        
        position_constraint.constraint_region = BoundingVolume()
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.03]
        position_constraint.constraint_region.primitives.append(sphere)
        
        sphere_pose = Pose()
        sphere_pose.position.x = x
        sphere_pose.position.y = y
        sphere_pose.position.z = z
        position_constraint.constraint_region.primitive_poses.append(sphere_pose)
        
        position_constraint.weight = 1.0
        constraints.position_constraints.append(position_constraint)
        
        return constraints
    
    def execute_arm_motion(self, goal_msg, description="执行运动"):
        """执行机械臂运动规划"""
        self.get_logger().info(f'发送规划请求：{description}...')
        
        before_positions = self.get_current_joint_positions()
        
        future = self.arm_move_group_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        
        if not future.done():
            self.get_logger().error('✗ 请求超时')
            return False
        
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('✗ 请求被拒绝')
            return False
        
        self.get_logger().info('✓ 请求已接受，执行中...')
        
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=15.0)
        
        if not result_future.done():
            self.get_logger().error('✗ 执行超时')
            return False
        
        # 检查运动结果
        after_positions = self.get_current_joint_positions()
        
        if before_positions and after_positions:
            changes = []
            for i, (before, after) in enumerate(zip(before_positions, after_positions)):
                change = abs(after - before)
                if change > 0.01:
                    changes.append((i, before, after, change))
            
            if changes:
                self.get_logger().info(f'✓ 运动执行成功！')
                for i, before, after, change in changes[:3]:
                    self.get_logger().info(f'  关节{i}: {before:.3f} -> {after:.3f}')
                return True
            else:
                self.get_logger().warn('⚠ 未检测到关节运动')
                return False
        
        self.get_logger().info('✓ 运动执行完成')
        return True
    
    def go_to_joint_position(self, joint_values, name="关节位置"):
        """移动机械臂到指定关节角度"""
        if len(joint_values) != 6:
            self.get_logger().error(f'✗ 需要6个关节值，得到{len(joint_values)}个')
            return False
        
        # 检查关节限制
        for i, (value, (lower, upper)) in enumerate(zip(joint_values, self.joint_limits)):
            if value < lower - 0.01 or value > upper + 0.01:
                self.get_logger().error(f'✗ joint{i+1} 角度 {value:.3f} 超出限制 [{lower:.3f}, {upper:.3f}]')
                return False
        
        self.get_logger().info(f'移动到{name}: {[f"{j:.3f}" for j in joint_values]}')
        
        try:
            goal_msg = MoveGroup.Goal()
            request = MotionPlanRequest()
            request.group_name = self.planning_group
            request.allowed_planning_time = 5.0
            request.max_velocity_scaling_factor = 0.3
            request.max_acceleration_scaling_factor = 0.3
            
            request.start_state = RobotState()
            request.start_state.is_diff = True
            
            request.goal_constraints.append(
                self.create_joint_constraints(self.arm_joint_names, joint_values)
            )
            
            planning_options = PlanningOptions()
            planning_options.plan_only = False
            planning_options.look_around = False
            planning_options.replan = True
            planning_options.replan_delay = 2.0
            
            goal_msg.request = request
            goal_msg.planning_options = planning_options
            
            return self.execute_arm_motion(goal_msg, f"机械臂移动到{name}")
                
        except Exception as e:
            self.get_logger().error(f'✗ 移动失败: {e}')
            return False
    
    def go_to_position(self, x, y, z):
        """移动到指定XYZ坐标"""
        self.get_logger().info(f'移动到坐标: x={x:.3f}, y={y:.3f}, z={z:.3f}')
        
        try:
            goal_msg = MoveGroup.Goal()
            request = MotionPlanRequest()
            request.group_name = self.planning_group
            request.allowed_planning_time = 5.0
            request.max_velocity_scaling_factor = 0.3
            request.max_acceleration_scaling_factor = 0.3
            request.num_planning_attempts = 10
            
            request.start_state = RobotState()
            request.start_state.is_diff = True
            
            request.goal_constraints.append(self.create_position_constraints(x, y, z))
            
            planning_options = PlanningOptions()
            planning_options.plan_only = False
            planning_options.look_around = True
            planning_options.replan = True
            planning_options.replan_delay = 2.0
            
            goal_msg.request = request
            goal_msg.planning_options = planning_options
            
            return self.execute_arm_motion(goal_msg, f"移动到坐标 ({x:.2f}, {y:.2f}, {z:.2f})")
                
        except Exception as e:
            self.get_logger().error(f'✗ 移动到坐标失败: {e}')
            return False
    
    def go_to_home(self):
        """回到零位"""
        self.get_logger().info('返回零位...')
        return self.go_to_joint_position([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], "零位")
    
    # ========== 夹爪控制 ==========
    def control_gripper_via_trajectory(self, width, duration=1.0):
        """通过 joint_trajectory 控制夹爪"""
        try:
            traj_msg = JointTrajectory()
            traj_msg.header.stamp = self.get_clock().now().to_msg()
            traj_msg.joint_names = [self.gripper_joint_name]
            
            point = JointTrajectoryPoint()
            point.positions = [width]
            point.velocities = [0.1]
            point.accelerations = [0.1]
            point.time_from_start.sec = int(duration)
            point.time_from_start.nanosec = int((duration % 1) * 1e9)
            
            traj_msg.points.append(point)
            
            self.gripper_trajectory_pub.publish(traj_msg)
            self.get_logger().info(f'✓ 已通过 joint_trajectory 发送夹爪命令: {width:.3f}m')
            
            time.sleep(duration + 0.5)
            return True
            
        except Exception as e:
            self.get_logger().error(f'✗ joint_trajectory 控制失败: {e}')
            return False
    
    def control_gripper_via_action(self, width, duration=1.0):
        """通过 action 控制夹爪"""
        if not self.gripper_action_client:
            return False
        
        try:
            goal = FollowJointTrajectory.Goal()
            goal.trajectory = JointTrajectory()
            goal.trajectory.header.stamp = self.get_clock().now().to_msg()
            goal.trajectory.joint_names = [self.gripper_joint_name]
            
            point = JointTrajectoryPoint()
            point.positions = [width]
            point.velocities = [0.1]
            point.time_from_start.sec = int(duration)
            point.time_from_start.nanosec = int((duration % 1) * 1e9)
            
            goal.trajectory.points.append(point)
            
            future = self.gripper_action_client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
            
            if not future.done():
                return False
            
            goal_handle = future.result()
            if not goal_handle.accepted:
                return False
            
            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future, timeout_sec=duration + 2.0)
            
            if result_future.done():
                self.get_logger().info(f'✓ 已通过 action 发送夹爪命令: {width:.3f}m')
                return True
            
            return False
                
        except Exception as e:
            return False
    
    def open_gripper(self, width=0.08):
        """打开夹爪"""
        self.get_logger().info(f'打开夹爪到 {width:.3f}m')
        
        # 优先使用 trajectory 控制
        if self.control_gripper_via_trajectory(width):
            self.gripper_force = 1.0
            return True
        elif self.control_gripper_via_action(width):
            self.gripper_force = 1.0
            return True
        else:
            self.get_logger().error('✗ 夹爪控制失败')
            return False
    
    def close_gripper(self, width=0.02, force=1.5):
        """关闭夹爪"""
        self.get_logger().info(f'关闭夹爪到 {width:.3f}m，力={force:.1f}N')
        self.gripper_force = force
        
        if self.control_gripper_via_trajectory(width):
            return True
        elif self.control_gripper_via_action(width):
            return True
        else:
            self.get_logger().error('✗ 夹爪控制失败')
            return False
    
    def set_gripper_width(self, width, force=1.0):
        """设置夹爪宽度"""
        self.get_logger().info(f'设置夹爪宽度: {width:.3f}m，力={force:.1f}N')
        self.gripper_force = force
        return self.open_gripper(width)
    
    # ========== 组合动作序列 ==========
    def pick_and_place_sequence(self):
        """完整的抓取和放置序列"""
        self.get_logger().info('\n' + '='*60)
        self.get_logger().info('开始抓取和放置序列')
        self.get_logger().info('='*60)
        
        # 1. 初始化 - 打开夹爪
        self.get_logger().info('\n步骤 1: 打开夹爪')
        self.open_gripper(0.08)
        time.sleep(0.5)
        
        # 2. 移动到抓取预备位置
        self.get_logger().info('\n步骤 2: 移动到抓取预备位置')
        pick_approach = [0.0, 0.5, -0.5, 0.0, 0.0, 0.0]
        self.go_to_joint_position(pick_approach, "抓取预备位置")
        time.sleep(0.5)
        
        # 3. 下降到抓取位置
        self.get_logger().info('\n步骤 3: 下降到抓取位置')
        pick_pose = [0.0, 0.8, -0.8, 0.0, 0.0, 0.0]
        self.go_to_joint_position(pick_pose, "抓取位置")
        time.sleep(0.5)
        
        # 4. 关闭夹爪抓取物体
        self.get_logger().info('\n步骤 4: 关闭夹爪抓取物体')
        self.close_gripper(0.03, force=1.5)
        time.sleep(1.0)
        
        # 5. 抬起物体
        self.get_logger().info('\n步骤 5: 抬起物体')
        self.go_to_joint_position(pick_approach, "抬起位置")
        time.sleep(0.5)
        
        # 6. 移动到放置预备位置（向左转）
        self.get_logger().info('\n步骤 6: 移动到放置预备位置')
        place_approach = [0.8, 0.5, -0.5, 0.0, 0.0, 0.0]
        self.go_to_joint_position(place_approach, "放置预备位置")
        time.sleep(0.5)
        
        # 7. 下降到放置位置
        self.get_logger().info('\n步骤 7: 下降到放置位置')
        place_pose = [0.8, 0.8, -0.8, 0.0, 0.0, 0.0]
        self.go_to_joint_position(place_pose, "放置位置")
        time.sleep(0.5)
        
        # 8. 打开夹爪释放物体
        self.get_logger().info('\n步骤 8: 打开夹爪释放物体')
        self.open_gripper(0.08)
        time.sleep(1.0)
        
        # 9. 回到预备位置
        self.get_logger().info('\n步骤 9: 回到预备位置')
        self.go_to_joint_position(place_approach, "预备位置")
        time.sleep(0.5)
        
        # 10. 回到零位
        self.get_logger().info('\n步骤 10: 返回零位')
        self.go_to_home()
        
        self.get_logger().info('\n✓ 抓取和放置序列完成!')
    
    def gripper_test_sequence(self):
        """夹爪测试序列"""
        self.get_logger().info('\n' + '='*60)
        self.get_logger().info('开始夹爪测试序列')
        self.get_logger().info('='*60)
        
        # 完全打开
        self.get_logger().info('\n测试 1: 完全打开 (0.1m)')
        self.open_gripper(0.1)
        time.sleep(2.0)
        
        # 完全关闭
        self.get_logger().info('\n测试 2: 完全关闭 (0.0m)')
        self.close_gripper(0.0, force=1.0)
        time.sleep(2.0)
        
        # 半开
        self.get_logger().info('\n测试 3: 半开 (0.05m)')
        self.set_gripper_width(0.05, 1.0)
        time.sleep(2.0)
        
        # 小开口
        self.get_logger().info('\n测试 4: 小开口 (0.02m)')
        self.set_gripper_width(0.02, 2.0)
        time.sleep(2.0)
        
        # 返回打开
        self.get_logger().info('\n测试 5: 返回打开状态 (0.08m)')
        self.open_gripper(0.08)
        time.sleep(2.0)
        
        self.get_logger().info('\n✓ 夹爪测试序列完成!')
    
    # ========== 状态显示 ==========
    def print_current_state(self):
        """打印当前状态"""
        self.get_logger().info('='*50)
        self.get_logger().info('当前状态:')
        
        if self.current_joint_positions:
            self.get_logger().info(f'机械臂关节: {[f"{p:.3f}" for p in self.current_joint_positions]}')
        else:
            self.get_logger().info('机械臂关节: 未知')
        
        if self.current_gripper_position is not None:
            self.get_logger().info(f'夹爪宽度: {self.current_gripper_position:.3f}m')
        else:
            self.get_logger().info('夹爪宽度: 未知')
        
        self.get_logger().info('='*50)


def main():
    rclpy.init()
    
    try:
        demo = MoveItGripperDemo()
        
        print("\n" + "="*60)
        print("Piper 机械臂 + AGX Gripper 控制 Demo")
        print("="*60)
        
        # 等待初始化
        print("等待获取状态...")
        time.sleep(2)
        
        demo.print_current_state()
        
        # 演示菜单
        while True:
            print("\n" + "="*60)
            print("请选择演示项目:")
            print("1. 夹爪测试序列")
            print("2. 完整抓取和放置序列")
            print("3. 手动控制夹爪")
            print("4. 手动控制机械臂关节")
            print("5. 回到零位")
            print("6. 打印状态")
            print("0. 退出")
            print("="*60)
            
            choice = input("请输入选项 (0-6): ").strip()
            
            if choice == '0':
                print("退出演示...")
                break
            
            elif choice == '1':
                input("\n按 Enter 键开始夹爪测试...")
                demo.gripper_test_sequence()
            
            elif choice == '2':
                input("\n按 Enter 键开始抓取和放置演示...")
                demo.pick_and_place_sequence()
            
            elif choice == '3':
                print("\n手动控制夹爪")
                print("输入宽度 (0.0-0.1米) 和力 (0.5-3.0N)")
                try:
                    width = float(input("宽度 (米): ").strip())
                    force = float(input("力 (牛顿, 回车默认1.0): ").strip() or "1.0")
                    
                    if 0.0 <= width <= 0.1 and 0.5 <= force <= 3.0:
                        demo.set_gripper_width(width, force)
                    else:
                        print("✗ 参数超出范围!")
                except ValueError:
                    print("✗ 输入无效!")
            
            elif choice == '4':
                print("\n手动控制机械臂关节")
                print("输入6个关节角度（弧度），用空格分隔")
                print("关节限制:")
                for i, (lower, upper) in enumerate(demo.joint_limits):
                    print(f"  joint{i+1}: [{lower:.2f}, {upper:.2f}]")
                
                try:
                    input_str = input("关节角度: ").strip()
                    values = [float(v) for v in input_str.split()]
                    
                    if len(values) == 6:
                        demo.go_to_joint_position(values, "手动位置")
                    else:
                        print(f"✗ 需要6个值，得到{len(values)}个")
                except ValueError:
                    print("✗ 输入无效!")
            
            elif choice == '5':
                demo.go_to_home()
            
            elif choice == '6':
                demo.print_current_state()
            
            else:
                print("无效选项，请重新输入")
        
        print("\n✓ Demo 完成!")
        
    except KeyboardInterrupt:
        print("\n用户中断")
    except Exception as e:
        print(f"\n✗ 发生错误: {e}")
        import traceback
        traceback.print_exc()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
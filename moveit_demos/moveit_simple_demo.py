#!/usr/bin/env python3
"""
MoveIt 简单控制 Demo - 使用标准 MoveIt Commander API
功能：基于 moveit_commander 的机械臂控制
"""

import sys
import rclpy
from rclpy.node import Node
import moveit_commander
from geometry_msgs.msg import Pose, PoseStamped
from tf_transformations import quaternion_from_euler
import math


class MoveItSimpleDemo(Node):
    def __init__(self):
        super().__init__('moveit_simple_demo')
        
        self.get_logger().info('初始化 MoveIt Commander...')
        
        # 初始化 moveit_commander
        moveit_commander.roscpp_initialize(sys.argv)
        
        # 创建机器人接口
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        
        # 创建机械臂规划组接口
        self.arm_group = moveit_commander.MoveGroupCommander('arm')
        
        # 设置规划参数
        self.arm_group.set_max_velocity_scaling_factor(0.3)
        self.arm_group.set_max_acceleration_scaling_factor(0.3)
        self.arm_group.set_planning_time(10.0)
        
        # 设置参考坐标系
        self.arm_group.set_pose_reference_frame('base_link')
        
        # 获取末端执行器名称
        self.eef_link = self.arm_group.get_end_effector_link()
        
        self.get_logger().info(f'规划组: {self.arm_group.get_name()}')
        self.get_logger().info(f'末端执行器: {self.eef_link}')
        self.get_logger().info(f'参考坐标系: {self.arm_group.get_planning_frame()}')
        self.get_logger().info('MoveIt 初始化完成!')
    
    def go_to_home(self):
        """回到零位"""
        self.get_logger().info('移动到零位 (home)...')
        self.arm_group.set_named_target('home')
        success = self.arm_group.go(wait=True)
        self.arm_group.stop()
        self.arm_group.clear_pose_targets()
        
        if success:
            self.get_logger().info('✓ 成功到达零位')
        else:
            self.get_logger().error('✗ 移动到零位失败')
        return success
    
    def go_to_joint_position(self, joint_values):
        """
        移动到指定关节角度
        
        Args:
            joint_values: 关节角度列表 [j1, j2, j3, j4, j5, j6] (弧度)
        """
        self.get_logger().info(f'移动到关节位置: {[f"{j:.3f}" for j in joint_values]}')
        self.arm_group.set_joint_value_target(joint_values)
        
        # 规划并执行
        success = self.arm_group.go(wait=True)
        self.arm_group.stop()
        
        if success:
            self.get_logger().info('✓ 成功到达目标关节位置')
        else:
            self.get_logger().error('✗ 移动失败')
        return success
    
    def go_to_pose_target(self, x, y, z, roll=0.0, pitch=0.0, yaw=0.0):
        """
        移动到指定笛卡尔空间位姿
        
        Args:
            x, y, z: 位置 (米)
            roll, pitch, yaw: 姿态 (弧度)
        """
        self.get_logger().info(f'移动到位姿: x={x:.3f}, y={y:.3f}, z={z:.3f}')
        
        # 创建目标位姿
        target_pose = Pose()
        target_pose.position.x = x
        target_pose.position.y = y
        target_pose.position.z = z
        
        # 将欧拉角转换为四元数
        q = quaternion_from_euler(roll, pitch, yaw)
        target_pose.orientation.x = q[0]
        target_pose.orientation.y = q[1]
        target_pose.orientation.z = q[2]
        target_pose.orientation.w = q[3]
        
        self.arm_group.set_pose_target(target_pose)
        
        # 规划并执行
        success = self.arm_group.go(wait=True)
        self.arm_group.stop()
        self.arm_group.clear_pose_targets()
        
        if success:
            self.get_logger().info('✓ 成功到达目标位姿')
        else:
            self.get_logger().error('✗ 移动失败')
        return success
    
    def plan_and_execute_cartesian_path(self, waypoints, eef_step=0.01):
        """
        规划并执行笛卡尔空间路径
        
        Args:
            waypoints: Pose 对象列表
            eef_step: 末端执行器步进大小 (米)
        """
        self.get_logger().info(f'规划笛卡尔路径，包含 {len(waypoints)} 个路径点...')
        
        # 规划笛卡尔路径
        (plan, fraction) = self.arm_group.compute_cartesian_path(
            waypoints,  # waypoints
            eef_step,   # eef_step
            0.0         # jump_threshold
        )
        
        self.get_logger().info(f'路径完成度: {fraction*100:.1f}%')
        
        if fraction > 0.95:
            self.get_logger().info('执行笛卡尔路径...')
            success = self.arm_group.execute(plan, wait=True)
            if success:
                self.get_logger().info('✓ 路径执行成功')
            else:
                self.get_logger().error('✗ 路径执行失败')
            return success
        else:
            self.get_logger().warn('⚠ 笛卡尔路径规划不完整，取消执行')
            return False
    
    def print_current_state(self):
        """打印当前状态"""
        current_joints = self.arm_group.get_current_joint_values()
        current_pose = self.arm_group.get_current_pose().pose
        
        self.get_logger().info('===== 当前状态 =====')
        self.get_logger().info(f'关节角度: {[f"{j:.3f}" for j in current_joints]}')
        self.get_logger().info(f'末端位置: x={current_pose.position.x:.3f}, '
                              f'y={current_pose.position.y:.3f}, '
                              f'z={current_pose.position.z:.3f}')
        self.get_logger().info('===================')
    
    def shutdown(self):
        """清理资源"""
        moveit_commander.roscpp_shutdown()


def demo_basic_movements(demo):
    """基础移动演示"""
    print("\n" + "="*60)
    print("演示 1: 基础移动")
    print("="*60)
    
    # 打印当前状态
    demo.print_current_state()
    
    # 回到零位
    input("\n按 Enter 键移动到零位...")
    demo.go_to_home()
    demo.print_current_state()
    
    # 移动到指定关节角度
    input("\n按 Enter 键移动到关节位置 1...")
    demo.go_to_joint_position([0.0, 0.4, -0.6, 0.0, 0.2, 0.0])
    demo.print_current_state()
    
    # 移动到另一个关节角度
    input("\n按 Enter 键移动到关节位置 2...")
    demo.go_to_joint_position([0.5, 0.3, -0.5, 0.0, -0.3, 0.8])
    demo.print_current_state()
    
    # 回到零位
    input("\n按 Enter 键返回零位...")
    demo.go_to_home()


def demo_cartesian_path(demo):
    """笛卡尔路径演示"""
    print("\n" + "="*60)
    print("演示 2: 笛卡尔路径 - 画正方形")
    print("="*60)
    
    input("\n按 Enter 键开始...")
    
    # 先移动到起始位置
    demo.go_to_joint_position([0.0, 0.3, -0.5, 0.0, 0.0, 0.0])
    
    # 获取当前位姿作为起点
    start_pose = demo.arm_group.get_current_pose().pose
    
    # 创建正方形路径点
    side_length = 0.1  # 10cm边长
    waypoints = []
    
    # 点1: 右移
    pose1 = Pose()
    pose1.position.x = start_pose.position.x
    pose1.position.y = start_pose.position.y + side_length
    pose1.position.z = start_pose.position.z
    pose1.orientation = start_pose.orientation
    waypoints.append(pose1)
    
    # 点2: 上移
    pose2 = Pose()
    pose2.position.x = start_pose.position.x
    pose2.position.y = start_pose.position.y + side_length
    pose2.position.z = start_pose.position.z + side_length
    pose2.orientation = start_pose.orientation
    waypoints.append(pose2)
    
    # 点3: 左移
    pose3 = Pose()
    pose3.position.x = start_pose.position.x
    pose3.position.y = start_pose.position.y
    pose3.position.z = start_pose.position.z + side_length
    pose3.orientation = start_pose.orientation
    waypoints.append(pose3)
    
    # 点4: 回到起点
    pose4 = Pose()
    pose4.position.x = start_pose.position.x
    pose4.position.y = start_pose.position.y
    pose4.position.z = start_pose.position.z
    pose4.orientation = start_pose.orientation
    waypoints.append(pose4)
    
    # 执行笛卡尔路径
    demo.plan_and_execute_cartesian_path(waypoints, eef_step=0.01)
    
    # 回到零位
    input("\n按 Enter 键返回零位...")
    demo.go_to_home()


def demo_circle_path(demo):
    """圆形路径演示"""
    print("\n" + "="*60)
    print("演示 3: 笛卡尔路径 - 画圆")
    print("="*60)
    
    input("\n按 Enter 键开始...")
    
    # 先移动到合适位置
    demo.go_to_joint_position([0.0, 0.3, -0.5, 0.0, 0.0, 0.0])
    
    # 获取当前位姿
    current_pose = demo.arm_group.get_current_pose().pose
    
    # 圆心和半径
    center_y = current_pose.position.y
    center_z = current_pose.position.z
    radius = 0.05  # 5cm半径
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
    
    # 执行笛卡尔路径
    demo.plan_and_execute_cartesian_path(waypoints, eef_step=0.005)
    
    # 回到零位
    input("\n按 Enter 键返回零位...")
    demo.go_to_home()


def main():
    rclpy.init()
    
    try:
        demo = MoveItSimpleDemo()
        
        print("\n" + "="*60)
        print("MoveIt 简单控制 Demo")
        print("="*60)
        print("确保已启动机械臂控制节点:")
        print("ros2 launch agx_arm_ctrl start_single_agx_arm_moveit.launch.py \\")
        print("  can_port:=can0 arm_type:=piper")
        print("="*60)
        
        # 演示菜单
        while True:
            print("\n" + "="*60)
            print("请选择演示项目:")
            print("1. 基础移动 (关节空间)")
            print("2. 画正方形 (笛卡尔路径)")
            print("3. 画圆形 (笛卡尔路径)")
            print("4. 回到零位")
            print("0. 退出")
            print("="*60)
            
            choice = input("请输入选项 (0-4): ").strip()
            
            if choice == '0':
                print("退出演示...")
                break
            elif choice == '1':
                demo_basic_movements(demo)
            elif choice == '2':
                demo_cartesian_path(demo)
            elif choice == '3':
                demo_circle_path(demo)
            elif choice == '4':
                demo.go_to_home()
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
        if 'demo' in locals():
            demo.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

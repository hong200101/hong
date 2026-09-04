#!/usr/bin/env python3
"""
MoveIt 基础控制 Demo
功能：使用 MoveIt 控制机械臂到达预设位置
"""

import rclpy
from rclpy.node import Node
from moveit_py_utils import MoveItPyUtils
import sys


class MoveItBasicDemo(Node):
    def __init__(self):
        super().__init__('moveit_basic_demo')
        
        self.get_logger().info('初始化 MoveIt Basic Demo...')
        
        # 初始化 MoveIt 接口
        self.moveit = MoveItPyUtils(node=self)
        
        # 设置规划组为机械臂
        self.arm_group = self.moveit.move_group_interface('arm')
        
        # 设置规划参数
        self.arm_group.set_max_velocity_scaling_factor(0.3)  # 最大速度比例
        self.arm_group.set_max_acceleration_scaling_factor(0.3)  # 最大加速度比例
        self.arm_group.set_planning_time(10.0)  # 规划时间
        
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
        self.get_logger().info(f'移动到关节位置: {joint_values}')
        self.arm_group.set_joint_value_target(joint_values)
        success = self.arm_group.go(wait=True)
        self.arm_group.stop()
        
        if success:
            self.get_logger().info('✓ 成功到达目标关节位置')
        else:
            self.get_logger().error('✗ 移动失败')
        return success
    
    def go_to_pose(self, x, y, z, roll=0.0, pitch=0.0, yaw=0.0):
        """
        移动到指定笛卡尔空间位姿
        
        Args:
            x, y, z: 位置 (米)
            roll, pitch, yaw: 姿态 (弧度)
        """
        self.get_logger().info(f'移动到位姿: x={x}, y={y}, z={z}')
        
        from geometry_msgs.msg import Pose
        from tf_transformations import quaternion_from_euler
        
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
        success = self.arm_group.go(wait=True)
        self.arm_group.stop()
        self.arm_group.clear_pose_targets()
        
        if success:
            self.get_logger().info('✓ 成功到达目标位姿')
        else:
            self.get_logger().error('✗ 移动失败')
        return success
    
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


def main():
    rclpy.init()
    
    try:
        demo = MoveItBasicDemo()
        
        print("\n" + "="*50)
        print("MoveIt 基础控制 Demo")
        print("="*50)
        
        # 打印当前状态
        demo.print_current_state()
        
        # 示例 1: 回到零位
        input("\n按 Enter 键移动到零位...")
        demo.go_to_home()
        demo.print_current_state()
        
        # 示例 2: 移动到指定关节角度 (Piper 6轴)
        input("\n按 Enter 键移动到关节位置 1...")
        demo.go_to_joint_position([0.0, 0.4, -0.6, 0.0, 0.0, 0.0])
        demo.print_current_state()
        
        # 示例 3: 移动到另一个关节角度
        input("\n按 Enter 键移动到关节位置 2...")
        demo.go_to_joint_position([0.5, 0.2, -0.4, 0.0, -0.5, 0.5])
        demo.print_current_state()
        
        # 示例 4: 使用笛卡尔空间规划
        input("\n按 Enter 键移动到笛卡尔位姿...")
        demo.go_to_pose(x=0.3, y=0.0, z=0.3, roll=0.0, pitch=0.0, yaw=0.0)
        demo.print_current_state()
        
        # 回到零位
        input("\n按 Enter 键返回零位...")
        demo.go_to_home()
        
        print("\n✓ Demo 完成!")
        
    except KeyboardInterrupt:
        print("\n用户中断")
    except Exception as e:
        print(f"\n✗ 发生错误: {e}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()

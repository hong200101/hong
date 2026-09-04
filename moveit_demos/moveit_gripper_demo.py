#!/usr/bin/env python3
"""
MoveIt 机械臂 + 夹爪控制 Demo
功能：结合 MoveIt 和夹爪，实现完整的抓取和放置动作
"""

import sys
import rclpy
from rclpy.node import Node
import moveit_commander
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from tf_transformations import quaternion_from_euler
import time


class MoveItGripperDemo(Node):
    def __init__(self):
        super().__init__('moveit_gripper_demo')
        
        self.get_logger().info('初始化 MoveIt Gripper Demo...')
        
        # 初始化 moveit_commander
        moveit_commander.roscpp_initialize(sys.argv)
        
        # 创建机器人接口
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        
        # 创建机械臂规划组接口
        self.arm_group = moveit_commander.MoveGroupCommander('arm')
        self.arm_group.set_max_velocity_scaling_factor(0.3)
        self.arm_group.set_max_acceleration_scaling_factor(0.3)
        self.arm_group.set_planning_time(10.0)
        
        # 尝试创建夹爪规划组接口
        try:
            self.gripper_group = moveit_commander.MoveGroupCommander('gripper')
            self.has_gripper = True
            self.get_logger().info('✓ 夹爪规划组已找到')
        except:
            self.has_gripper = False
            self.get_logger().warn('⚠ 未找到夹爪规划组，将使用话题控制')
            # 创建夹爪控制发布者
            self.gripper_pub = self.create_publisher(
                JointState,
                '/control/joint_states',
                10
            )
        
        self.get_logger().info(f'机械臂规划组: {self.arm_group.get_name()}')
        self.get_logger().info(f'末端执行器: {self.arm_group.get_end_effector_link()}')
        self.get_logger().info('MoveIt 初始化完成!')
    
    def open_gripper(self, width=0.1, force=1.0):
        """
        打开夹爪
        
        Args:
            width: 目标宽度 (米)，范围 [0.0, 0.1]
            force: 夹持力 (牛顿)，范围 [0.5, 3.0]
        """
        self.get_logger().info(f'打开夹爪: 宽度={width}m, 力={force}N')
        
        if self.has_gripper:
            # 使用 MoveIt 控制夹爪
            self.gripper_group.set_named_target('gripper_open')
            success = self.gripper_group.go(wait=True)
            self.gripper_group.stop()
            return success
        else:
            # 使用话题控制夹爪
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = ['gripper']
            msg.position = [width]
            msg.effort = [force]
            
            self.gripper_pub.publish(msg)
            time.sleep(1.0)  # 等待夹爪动作完成
            return True
    
    def close_gripper(self, width=0.0, force=1.0):
        """
        关闭夹爪
        
        Args:
            width: 目标宽度 (米)，范围 [0.0, 0.1]
            force: 夹持力 (牛顿)，范围 [0.5, 3.0]
        """
        self.get_logger().info(f'关闭夹爪: 宽度={width}m, 力={force}N')
        
        if self.has_gripper:
            # 使用 MoveIt 控制夹爪
            self.gripper_group.set_named_target('gripper_close')
            success = self.gripper_group.go(wait=True)
            self.gripper_group.stop()
            return success
        else:
            # 使用话题控制夹爪
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = ['gripper']
            msg.position = [width]
            msg.effort = [force]
            
            self.gripper_pub.publish(msg)
            time.sleep(1.0)  # 等待夹爪动作完成
            return True
    
    def set_gripper_width(self, width, force=1.0):
        """
        设置夹爪宽度
        
        Args:
            width: 目标宽度 (米)，范围 [0.0, 0.1]
            force: 夹持力 (牛顿)，范围 [0.5, 3.0]
        """
        self.get_logger().info(f'设置夹爪宽度: {width}m, 力={force}N')
        
        if self.has_gripper:
            # 使用 MoveIt 控制
            joint_goal = self.gripper_group.get_current_joint_values()
            joint_goal[0] = width
            self.gripper_group.set_joint_value_target(joint_goal)
            success = self.gripper_group.go(wait=True)
            self.gripper_group.stop()
            return success
        else:
            # 使用话题控制
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = ['gripper']
            msg.position = [width]
            msg.effort = [force]
            
            self.gripper_pub.publish(msg)
            time.sleep(1.0)
            return True
    
    def go_to_home(self):
        """回到零位"""
        self.get_logger().info('返回零位...')
        self.arm_group.set_named_target('home')
        success = self.arm_group.go(wait=True)
        self.arm_group.stop()
        self.arm_group.clear_pose_targets()
        return success
    
    def go_to_joint_position(self, joint_values):
        """移动到指定关节角度"""
        self.get_logger().info(f'移动到关节位置: {[f"{j:.3f}" for j in joint_values]}')
        self.arm_group.set_joint_value_target(joint_values)
        success = self.arm_group.go(wait=True)
        self.arm_group.stop()
        return success
    
    def pick_and_place_sequence(self):
        """完整的抓取和放置序列"""
        self.get_logger().info('\n' + '='*60)
        self.get_logger().info('开始抓取和放置序列')
        self.get_logger().info('='*60)
        
        # 1. 初始化 - 打开夹爪
        self.get_logger().info('\n步骤 1: 打开夹爪')
        self.open_gripper(width=0.08)
        time.sleep(0.5)
        
        # 2. 移动到抓取预备位置
        self.get_logger().info('\n步骤 2: 移动到抓取预备位置')
        pick_approach = [0.0, 0.3, -0.5, 0.0, 0.2, 0.0]
        self.go_to_joint_position(pick_approach)
        time.sleep(0.5)
        
        # 3. 下降到抓取位置
        self.get_logger().info('\n步骤 3: 下降到抓取位置')
        pick_pose = [0.0, 0.5, -0.7, 0.0, 0.2, 0.0]
        self.go_to_joint_position(pick_pose)
        time.sleep(0.5)
        
        # 4. 关闭夹爪抓取物体
        self.get_logger().info('\n步骤 4: 关闭夹爪抓取物体')
        self.close_gripper(width=0.03, force=1.5)
        time.sleep(1.0)
        
        # 5. 抬起物体
        self.get_logger().info('\n步骤 5: 抬起物体')
        self.go_to_joint_position(pick_approach)
        time.sleep(0.5)
        
        # 6. 移动到放置预备位置
        self.get_logger().info('\n步骤 6: 移动到放置预备位置')
        place_approach = [0.8, 0.3, -0.5, 0.0, 0.2, 0.5]
        self.go_to_joint_position(place_approach)
        time.sleep(0.5)
        
        # 7. 下降到放置位置
        self.get_logger().info('\n步骤 7: 下降到放置位置')
        place_pose = [0.8, 0.5, -0.7, 0.0, 0.2, 0.5]
        self.go_to_joint_position(place_pose)
        time.sleep(0.5)
        
        # 8. 打开夹爪释放物体
        self.get_logger().info('\n步骤 8: 打开夹爪释放物体')
        self.open_gripper(width=0.08)
        time.sleep(1.0)
        
        # 9. 回到预备位置
        self.get_logger().info('\n步骤 9: 回到预备位置')
        self.go_to_joint_position(place_approach)
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
        self.open_gripper(width=0.1, force=1.0)
        time.sleep(2.0)
        
        # 完全关闭
        self.get_logger().info('\n测试 2: 完全关闭 (0.0m)')
        self.close_gripper(width=0.0, force=1.0)
        time.sleep(2.0)
        
        # 半开
        self.get_logger().info('\n测试 3: 半开 (0.05m)')
        self.set_gripper_width(width=0.05, force=1.0)
        time.sleep(2.0)
        
        # 小开口（模拟抓取小物体）
        self.get_logger().info('\n测试 4: 小开口 (0.02m)')
        self.set_gripper_width(width=0.02, force=2.0)
        time.sleep(2.0)
        
        # 回到打开状态
        self.get_logger().info('\n测试 5: 返回打开状态 (0.08m)')
        self.open_gripper(width=0.08, force=1.0)
        time.sleep(2.0)
        
        self.get_logger().info('\n✓ 夹爪测试序列完成!')
    
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


def main():
    rclpy.init()
    
    try:
        demo = MoveItGripperDemo()
        
        print("\n" + "="*60)
        print("MoveIt 机械臂 + 夹爪控制 Demo")
        print("="*60)
        print("确保已启动机械臂控制节点（带夹爪）:")
        print("ros2 launch agx_arm_ctrl start_single_agx_arm_moveit.launch.py \\")
        print("  can_port:=can0 arm_type:=piper effector_type:=agx_gripper")
        print("="*60)
        
        # 演示菜单
        while True:
            print("\n" + "="*60)
            print("请选择演示项目:")
            print("1. 夹爪测试序列")
            print("2. 完整抓取和放置序列")
            print("3. 手动控制夹爪")
            print("4. 回到零位")
            print("0. 退出")
            print("="*60)
            
            choice = input("请输入选项 (0-4): ").strip()
            
            if choice == '0':
                print("退出演示...")
                break
            
            elif choice == '1':
                # 夹爪测试
                input("\n按 Enter 键开始夹爪测试...")
                demo.gripper_test_sequence()
            
            elif choice == '2':
                # 完整抓取和放置
                input("\n按 Enter 键开始抓取和放置演示...")
                demo.pick_and_place_sequence()
            
            elif choice == '3':
                # 手动控制夹爪
                print("\n手动控制夹爪")
                print("输入宽度 (0.0-0.1米) 和力 (0.5-3.0N)")
                try:
                    width = float(input("宽度 (米): ").strip())
                    force = float(input("力 (牛顿): ").strip())
                    
                    if 0.0 <= width <= 0.1 and 0.5 <= force <= 3.0:
                        demo.set_gripper_width(width, force)
                    else:
                        print("✗ 参数超出范围!")
                except ValueError:
                    print("✗ 输入无效!")
            
            elif choice == '4':
                # 回到零位
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

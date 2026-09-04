#!/usr/bin/env python3
"""
MoveIt + Gazebo 仿真控制 Demo
功能：在 Gazebo 仿真环境中使用 MoveIt 控制机械臂
"""

import sys
import rclpy
from rclpy.node import Node
import moveit_commander
from geometry_msgs.msg import Pose
from tf_transformations import quaternion_from_euler
import math
import time


class MoveItGazeboDemo(Node):
    def __init__(self):
        super().__init__('moveit_gazebo_demo')
        
        self.get_logger().info('初始化 MoveIt Gazebo Demo...')
        
        # 初始化 moveit_commander
        moveit_commander.roscpp_initialize(sys.argv)
        
        # 创建机器人接口
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        
        # 创建机械臂规划组接口
        self.arm_group = moveit_commander.MoveGroupCommander('arm')
        
        # 设置规划参数（Gazebo 仿真可以使用更高的速度）
        self.arm_group.set_max_velocity_scaling_factor(0.5)
        self.arm_group.set_max_acceleration_scaling_factor(0.5)
        self.arm_group.set_planning_time(10.0)
        
        # 设置参考坐标系
        self.arm_group.set_pose_reference_frame('base_link')
        
        # 获取末端执行器名称
        self.eef_link = self.arm_group.get_end_effector_link()
        
        # 尝试创建夹爪规划组（如果有）
        try:
            self.gripper_group = moveit_commander.MoveGroupCommander('gripper')
            self.has_gripper = True
            self.get_logger().info('✓ 夹爪规划组已找到')
        except:
            self.has_gripper = False
            self.get_logger().info('ℹ 未找到夹爪规划组')
        
        self.get_logger().info(f'规划组: {self.arm_group.get_name()}')
        self.get_logger().info(f'末端执行器: {self.eef_link}')
        self.get_logger().info(f'参考坐标系: {self.arm_group.get_planning_frame()}')
        self.get_logger().info('MoveIt Gazebo Demo 初始化完成!')
        self.get_logger().info('✓ 在 Gazebo 仿真环境中运行')
    
    def wait_for_gazebo(self):
        """等待 Gazebo 就绪"""
        self.get_logger().info('等待 Gazebo 仿真环境就绪...')
        time.sleep(2.0)
        self.get_logger().info('✓ Gazebo 已就绪')
    
    def add_table_to_scene(self):
        """添加桌面到场景中（避障）"""
        self.get_logger().info('添加桌面到场景...')
        
        table_pose = Pose()
        table_pose.position.x = 0.0
        table_pose.position.y = 0.0
        table_pose.position.z = -0.05
        table_pose.orientation.w = 1.0
        
        self.scene.add_box('table', table_pose, size=(1.0, 1.0, 0.1))
        time.sleep(0.5)
        self.get_logger().info('✓ 桌面已添加')
    
    def add_obstacle(self, name, x, y, z, size=(0.1, 0.1, 0.3)):
        """添加障碍物到场景"""
        self.get_logger().info(f'添加障碍物: {name}')
        
        obstacle_pose = Pose()
        obstacle_pose.position.x = x
        obstacle_pose.position.y = y
        obstacle_pose.position.z = z
        obstacle_pose.orientation.w = 1.0
        
        self.scene.add_box(name, obstacle_pose, size=size)
        time.sleep(0.5)
        self.get_logger().info(f'✓ 障碍物 {name} 已添加')
    
    def remove_all_objects(self):
        """移除所有场景对象"""
        self.get_logger().info('清除场景中的所有对象...')
        
        known_objects = self.scene.get_known_object_names()
        for obj in known_objects:
            self.scene.remove_world_object(obj)
        
        time.sleep(0.5)
        self.get_logger().info('✓ 场景已清空')
    
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
        """移动到指定关节角度"""
        self.get_logger().info(f'移动到关节位置: {[f"{j:.3f}" for j in joint_values]}')
        self.arm_group.set_joint_value_target(joint_values)
        success = self.arm_group.go(wait=True)
        self.arm_group.stop()
        
        if success:
            self.get_logger().info('✓ 成功到达目标关节位置')
        else:
            self.get_logger().error('✗ 移动失败')
        return success
    
    def go_to_pose_target(self, x, y, z, roll=0.0, pitch=0.0, yaw=0.0):
        """移动到指定笛卡尔空间位姿"""
        self.get_logger().info(f'移动到位姿: x={x:.3f}, y={y:.3f}, z={z:.3f}')
        
        target_pose = Pose()
        target_pose.position.x = x
        target_pose.position.y = y
        target_pose.position.z = z
        
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
    
    def plan_and_execute_cartesian_path(self, waypoints, eef_step=0.01):
        """规划并执行笛卡尔空间路径"""
        self.get_logger().info(f'规划笛卡尔路径，包含 {len(waypoints)} 个路径点...')
        
        (plan, fraction) = self.arm_group.compute_cartesian_path(
            waypoints,
            eef_step,
            0.0
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
    
    def draw_circle_in_gazebo(self, center_x=0.3, center_y=0.0, center_z=0.3, radius=0.05, num_points=30):
        """在 Gazebo 中画圆"""
        self.get_logger().info(f'在 Gazebo 中画圆：圆心=({center_x}, {center_y}, {center_z}), 半径={radius}')
        
        # 先移动到起始位置
        self.go_to_joint_position([0.0, 0.3, -0.5, 0.0, 0.0, 0.0])
        time.sleep(0.5)
        
        # 获取当前位姿
        current_pose = self.arm_group.get_current_pose().pose
        
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
        
        return self.plan_and_execute_cartesian_path(waypoints, eef_step=0.005)
    
    def draw_square_in_gazebo(self, start_x=0.3, start_y=-0.05, start_z=0.25, side_length=0.1):
        """在 Gazebo 中画正方形"""
        self.get_logger().info(f'在 Gazebo 中画正方形：起点=({start_x}, {start_y}, {start_z}), 边长={side_length}')
        
        # 先移动到起始位置
        self.go_to_joint_position([0.0, 0.3, -0.5, 0.0, 0.0, 0.0])
        time.sleep(0.5)
        
        current_pose = self.arm_group.get_current_pose().pose
        
        # 创建正方形路径点
        waypoints = []
        
        # 点1: 起点
        pose1 = Pose()
        pose1.position.x = start_x
        pose1.position.y = start_y
        pose1.position.z = start_z
        pose1.orientation = current_pose.orientation
        waypoints.append(pose1)
        
        # 点2: 右移
        pose2 = Pose()
        pose2.position.x = start_x
        pose2.position.y = start_y + side_length
        pose2.position.z = start_z
        pose2.orientation = current_pose.orientation
        waypoints.append(pose2)
        
        # 点3: 上移
        pose3 = Pose()
        pose3.position.x = start_x
        pose3.position.y = start_y + side_length
        pose3.position.z = start_z + side_length
        pose3.orientation = current_pose.orientation
        waypoints.append(pose3)
        
        # 点4: 左移
        pose4 = Pose()
        pose4.position.x = start_x
        pose4.position.y = start_y
        pose4.position.z = start_z + side_length
        pose4.orientation = current_pose.orientation
        waypoints.append(pose4)
        
        # 点5: 回到起点
        waypoints.append(pose1)
        
        return self.plan_and_execute_cartesian_path(waypoints, eef_step=0.01)
    
    def gazebo_pick_and_place_demo(self):
        """Gazebo 仿真中的抓取和放置演示"""
        self.get_logger().info('\n' + '='*60)
        self.get_logger().info('Gazebo 抓取和放置演示')
        self.get_logger().info('='*60)
        
        # 1. 移动到抓取预备位置
        self.get_logger().info('\n步骤 1: 移动到抓取预备位置')
        self.go_to_joint_position([0.0, 0.3, -0.5, 0.0, 0.2, 0.0])
        time.sleep(1.0)
        
        # 2. 下降到抓取位置
        self.get_logger().info('\n步骤 2: 下降到抓取位置')
        self.go_to_joint_position([0.0, 0.5, -0.7, 0.0, 0.2, 0.0])
        time.sleep(1.0)
        
        # 3. 模拟抓取
        self.get_logger().info('\n步骤 3: (模拟) 关闭夹爪抓取')
        if self.has_gripper:
            self.gripper_group.set_named_target('gripper_close')
            self.gripper_group.go(wait=True)
        time.sleep(1.0)
        
        # 4. 抬起
        self.get_logger().info('\n步骤 4: 抬起物体')
        self.go_to_joint_position([0.0, 0.3, -0.5, 0.0, 0.2, 0.0])
        time.sleep(1.0)
        
        # 5. 移动到放置位置
        self.get_logger().info('\n步骤 5: 移动到放置位置')
        self.go_to_joint_position([0.8, 0.3, -0.5, 0.0, 0.2, 0.5])
        time.sleep(1.0)
        
        # 6. 下降
        self.get_logger().info('\n步骤 6: 下降到放置位置')
        self.go_to_joint_position([0.8, 0.5, -0.7, 0.0, 0.2, 0.5])
        time.sleep(1.0)
        
        # 7. 模拟释放
        self.get_logger().info('\n步骤 7: (模拟) 打开夹爪释放')
        if self.has_gripper:
            self.gripper_group.set_named_target('gripper_open')
            self.gripper_group.go(wait=True)
        time.sleep(1.0)
        
        # 8. 回到预备位置
        self.get_logger().info('\n步骤 8: 回到预备位置')
        self.go_to_joint_position([0.8, 0.3, -0.5, 0.0, 0.2, 0.5])
        time.sleep(0.5)
        
        # 9. 回到零位
        self.get_logger().info('\n步骤 9: 返回零位')
        self.go_to_home()
        
        self.get_logger().info('\n✓ Gazebo 抓取和放置演示完成!')
    
    def obstacle_avoidance_demo(self):
        """避障演示"""
        self.get_logger().info('\n' + '='*60)
        self.get_logger().info('Gazebo 避障演示')
        self.get_logger().info('='*60)
        
        # 添加障碍物
        self.add_table_to_scene()
        self.add_obstacle('obstacle1', 0.3, 0.15, 0.2, size=(0.1, 0.1, 0.4))
        
        # 回到零位
        self.go_to_home()
        time.sleep(1.0)
        
        # 尝试移动到障碍物附近（MoveIt 会自动避障）
        self.get_logger().info('\n移动到障碍物附近（自动避障）...')
        self.go_to_pose_target(x=0.3, y=0.05, z=0.3)
        time.sleep(1.0)
        
        # 移动到障碍物另一侧
        self.get_logger().info('\n移动到障碍物另一侧...')
        self.go_to_pose_target(x=0.3, y=0.25, z=0.3)
        time.sleep(1.0)
        
        # 回到零位
        self.go_to_home()
        
        # 清除障碍物
        self.remove_all_objects()
        
        self.get_logger().info('\n✓ 避障演示完成!')
    
    def print_current_state(self):
        """打印当前状态"""
        current_joints = self.arm_group.get_current_joint_values()
        current_pose = self.arm_group.get_current_pose().pose
        
        self.get_logger().info('===== 当前状态 (Gazebo) =====')
        self.get_logger().info(f'关节角度: {[f"{j:.3f}" for j in current_joints]}')
        self.get_logger().info(f'末端位置: x={current_pose.position.x:.3f}, '
                              f'y={current_pose.position.y:.3f}, '
                              f'z={current_pose.position.z:.3f}')
        self.get_logger().info('===========================')
    
    def shutdown(self):
        """清理资源"""
        self.remove_all_objects()
        moveit_commander.roscpp_shutdown()


def main():
    rclpy.init()
    
    try:
        demo = MoveItGazeboDemo()
        
        print("\n" + "="*70)
        print("MoveIt + Gazebo 仿真控制 Demo")
        print("="*70)
        print("确保已启动 Gazebo 仿真环境:")
        print("ros2 launch agx_arm_moveit demo.launch.py arm_type:=piper use_sim_time:=true")
        print("或者参考文档中的 Gazebo 启动命令")
        print("="*70)
        
        # 等待 Gazebo 就绪
        demo.wait_for_gazebo()
        
        # 演示菜单
        while True:
            print("\n" + "="*70)
            print("请选择 Gazebo 演示项目:")
            print("="*70)
            print("1. 基础移动 (关节空间)")
            print("2. 在 Gazebo 中画圆形")
            print("3. 在 Gazebo 中画正方形")
            print("4. Gazebo 抓取和放置演示")
            print("5. Gazebo 避障演示")
            print("6. 添加/清除场景对象")
            print("7. 查看当前状态")
            print("8. 回到零位")
            print("0. 退出")
            print("="*70)
            
            choice = input("请输入选项 (0-8): ").strip()
            
            if choice == '0':
                print("退出演示...")
                break
            
            elif choice == '1':
                # 基础移动
                input("\n按 Enter 键开始基础移动演示...")
                demo.go_to_home()
                time.sleep(1.0)
                
                demo.go_to_joint_position([0.0, 0.4, -0.6, 0.0, 0.2, 0.0])
                time.sleep(1.0)
                
                demo.go_to_joint_position([0.5, 0.3, -0.5, 0.0, -0.3, 0.8])
                time.sleep(1.0)
                
                demo.go_to_home()
            
            elif choice == '2':
                # 画圆
                input("\n按 Enter 键在 Gazebo 中画圆...")
                demo.draw_circle_in_gazebo()
                demo.go_to_home()
            
            elif choice == '3':
                # 画正方形
                input("\n按 Enter 键在 Gazebo 中画正方形...")
                demo.draw_square_in_gazebo()
                demo.go_to_home()
            
            elif choice == '4':
                # 抓取和放置
                input("\n按 Enter 键开始 Gazebo 抓取和放置演示...")
                demo.gazebo_pick_and_place_demo()
            
            elif choice == '5':
                # 避障演示
                input("\n按 Enter 键开始避障演示...")
                demo.obstacle_avoidance_demo()
            
            elif choice == '6':
                # 场景对象管理
                print("\n场景对象管理:")
                print("1. 添加桌面")
                print("2. 添加障碍物")
                print("3. 清除所有对象")
                sub_choice = input("请选择 (1-3): ").strip()
                
                if sub_choice == '1':
                    demo.add_table_to_scene()
                elif sub_choice == '2':
                    demo.add_obstacle('test_obstacle', 0.3, 0.2, 0.3)
                elif sub_choice == '3':
                    demo.remove_all_objects()
            
            elif choice == '7':
                # 查看当前状态
                demo.print_current_state()
            
            elif choice == '8':
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

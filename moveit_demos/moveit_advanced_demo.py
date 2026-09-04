#!/usr/bin/env python3
"""
MoveIt 高级控制 Demo
功能：轨迹规划、笛卡尔路径、避障等高级功能
"""

import rclpy
from rclpy.node import Node
from moveit_py_utils import MoveItPyUtils
from geometry_msgs.msg import Pose
from tf_transformations import quaternion_from_euler
import math


class MoveItAdvancedDemo(Node):
    def __init__(self):
        super().__init__('moveit_advanced_demo')
        
        self.get_logger().info('初始化 MoveIt Advanced Demo...')
        
        # 初始化 MoveIt 接口
        self.moveit = MoveItPyUtils(node=self)
        self.arm_group = self.moveit.move_group_interface('arm')
        
        # 设置规划参数
        self.arm_group.set_max_velocity_scaling_factor(0.2)
        self.arm_group.set_max_acceleration_scaling_factor(0.2)
        self.arm_group.set_planning_time(10.0)
        
        # 设置位姿和关节容差
        self.arm_group.set_goal_position_tolerance(0.01)  # 1cm
        self.arm_group.set_goal_orientation_tolerance(0.05)  # 约3度
        
        self.get_logger().info('MoveIt 初始化完成!')
    
    def plan_cartesian_path(self, waypoints, eef_step=0.01, jump_threshold=0.0):
        """
        规划笛卡尔空间路径
        
        Args:
            waypoints: 路径点列表 [(x, y, z, roll, pitch, yaw), ...]
            eef_step: 末端执行器步进大小 (米)
            jump_threshold: 跳跃阈值
        """
        self.get_logger().info(f'规划笛卡尔路径，包含 {len(waypoints)} 个路径点...')
        
        # 创建 Pose 列表
        pose_list = []
        for wp in waypoints:
            pose = Pose()
            pose.position.x = wp[0]
            pose.position.y = wp[1]
            pose.position.z = wp[2]
            
            # 转换姿态
            if len(wp) >= 6:
                q = quaternion_from_euler(wp[3], wp[4], wp[5])
                pose.orientation.x = q[0]
                pose.orientation.y = q[1]
                pose.orientation.z = q[2]
                pose.orientation.w = q[3]
            else:
                pose.orientation.w = 1.0
            
            pose_list.append(pose)
        
        # 规划笛卡尔路径
        (plan, fraction) = self.arm_group.compute_cartesian_path(
            pose_list,
            eef_step,
            jump_threshold
        )
        
        self.get_logger().info(f'路径完成度: {fraction*100:.1f}%')
        
        if fraction > 0.95:
            self.get_logger().info('✓ 笛卡尔路径规划成功')
            return plan
        else:
            self.get_logger().warn('⚠ 笛卡尔路径规划不完整')
            return None
    
    def execute_plan(self, plan):
        """执行规划好的路径"""
        if plan is None:
            self.get_logger().error('✗ 无有效路径可执行')
            return False
        
        self.get_logger().info('执行规划路径...')
        success = self.arm_group.execute(plan, wait=True)
        
        if success:
            self.get_logger().info('✓ 路径执行成功')
        else:
            self.get_logger().error('✗ 路径执行失败')
        
        return success
    
    def draw_circle(self, center_x, center_y, center_z, radius, num_points=20):
        """
        在笛卡尔空间画圆
        
        Args:
            center_x, center_y, center_z: 圆心坐标 (米)
            radius: 半径 (米)
            num_points: 路径点数量
        """
        self.get_logger().info(f'规划圆形路径：圆心=({center_x}, {center_y}, {center_z}), 半径={radius}')
        
        waypoints = []
        for i in range(num_points + 1):
            angle = 2 * math.pi * i / num_points
            x = center_x + radius * math.cos(angle)
            y = center_y + radius * math.sin(angle)
            z = center_z
            waypoints.append((x, y, z, 0.0, 0.0, 0.0))
        
        return self.plan_cartesian_path(waypoints, eef_step=0.005)
    
    def draw_square(self, start_x, start_y, start_z, side_length):
        """
        在笛卡尔空间画正方形
        
        Args:
            start_x, start_y, start_z: 起点坐标 (米)
            side_length: 边长 (米)
        """
        self.get_logger().info(f'规划正方形路径：起点=({start_x}, {start_y}, {start_z}), 边长={side_length}')
        
        # 定义正方形的四个顶点
        waypoints = [
            (start_x, start_y, start_z, 0.0, 0.0, 0.0),
            (start_x + side_length, start_y, start_z, 0.0, 0.0, 0.0),
            (start_x + side_length, start_y + side_length, start_z, 0.0, 0.0, 0.0),
            (start_x, start_y + side_length, start_z, 0.0, 0.0, 0.0),
            (start_x, start_y, start_z, 0.0, 0.0, 0.0),  # 回到起点
        ]
        
        return self.plan_cartesian_path(waypoints, eef_step=0.01)
    
    def pick_and_place_demo(self):
        """简单的抓取和放置 Demo"""
        self.get_logger().info('开始抓取和放置演示...')
        
        # 1. 移动到抓取预备位置
        self.get_logger().info('移动到抓取预备位置...')
        pick_approach = [0.0, 0.3, -0.5, 0.0, 0.0, 0.0]
        self.arm_group.set_joint_value_target(pick_approach)
        self.arm_group.go(wait=True)
        self.arm_group.stop()
        
        # 2. 模拟下降抓取
        self.get_logger().info('下降到抓取位置...')
        pick_pose = [0.0, 0.5, -0.7, 0.0, 0.0, 0.0]
        self.arm_group.set_joint_value_target(pick_pose)
        self.arm_group.go(wait=True)
        self.arm_group.stop()
        
        # 这里应该关闭夹爪，但需要夹爪控制接口
        self.get_logger().info('(模拟) 关闭夹爪抓取物体')
        
        # 3. 抬起
        self.get_logger().info('抬起物体...')
        self.arm_group.set_joint_value_target(pick_approach)
        self.arm_group.go(wait=True)
        self.arm_group.stop()
        
        # 4. 移动到放置预备位置
        self.get_logger().info('移动到放置预备位置...')
        place_approach = [0.5, 0.3, -0.5, 0.0, -0.5, 0.5]
        self.arm_group.set_joint_value_target(place_approach)
        self.arm_group.go(wait=True)
        self.arm_group.stop()
        
        # 5. 下降放置
        self.get_logger().info('下降到放置位置...')
        place_pose = [0.5, 0.5, -0.7, 0.0, -0.5, 0.5]
        self.arm_group.set_joint_value_target(place_pose)
        self.arm_group.go(wait=True)
        self.arm_group.stop()
        
        # 这里应该打开夹爪
        self.get_logger().info('(模拟) 打开夹爪释放物体')
        
        # 6. 回到预备位置
        self.get_logger().info('回到预备位置...')
        self.arm_group.set_joint_value_target(place_approach)
        self.arm_group.go(wait=True)
        self.arm_group.stop()
        
        self.get_logger().info('✓ 抓取和放置演示完成')
    
    def go_to_home(self):
        """回到零位"""
        self.get_logger().info('返回零位...')
        self.arm_group.set_named_target('home')
        self.arm_group.go(wait=True)
        self.arm_group.stop()
        self.arm_group.clear_pose_targets()


def main():
    rclpy.init()
    
    try:
        demo = MoveItAdvancedDemo()
        
        print("\n" + "="*60)
        print("MoveIt 高级控制 Demo")
        print("="*60)
        
        # 回到零位
        print("\n准备开始演示...")
        input("按 Enter 键移动到零位...")
        demo.go_to_home()
        
        # 演示菜单
        while True:
            print("\n" + "="*60)
            print("请选择演示项目:")
            print("1. 画圆形")
            print("2. 画正方形")
            print("3. 抓取和放置演示")
            print("4. 自定义笛卡尔路径")
            print("0. 退出")
            print("="*60)
            
            choice = input("请输入选项 (0-4): ").strip()
            
            if choice == '0':
                print("退出演示...")
                break
            
            elif choice == '1':
                # 画圆
                input("\n按 Enter 键开始画圆...")
                plan = demo.draw_circle(
                    center_x=0.3,
                    center_y=0.0,
                    center_z=0.3,
                    radius=0.05,
                    num_points=30
                )
                if plan:
                    demo.execute_plan(plan)
            
            elif choice == '2':
                # 画正方形
                input("\n按 Enter 键开始画正方形...")
                plan = demo.draw_square(
                    start_x=0.25,
                    start_y=-0.05,
                    start_z=0.3,
                    side_length=0.1
                )
                if plan:
                    demo.execute_plan(plan)
            
            elif choice == '3':
                # 抓取和放置
                input("\n按 Enter 键开始抓取和放置演示...")
                demo.pick_and_place_demo()
            
            elif choice == '4':
                # 自定义路径
                print("\n自定义笛卡尔路径演示")
                waypoints = [
                    (0.3, 0.0, 0.2, 0.0, 0.0, 0.0),
                    (0.3, 0.1, 0.3, 0.0, 0.0, 0.0),
                    (0.3, 0.0, 0.4, 0.0, 0.0, 0.0),
                    (0.3, -0.1, 0.3, 0.0, 0.0, 0.0),
                    (0.3, 0.0, 0.2, 0.0, 0.0, 0.0),
                ]
                input("按 Enter 键执行自定义路径...")
                plan = demo.plan_cartesian_path(waypoints)
                if plan:
                    demo.execute_plan(plan)
            
            else:
                print("无效选项，请重新输入")
            
            # 回到零位
            input("\n按 Enter 键返回零位...")
            demo.go_to_home()
        
        print("\n✓ 演示完成!")
        
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

#!/usr/bin/env python3
"""
混合控制模块
功能：MoveIt 粗定位 + MIT 柔顺精调
"""

import sys
import rclpy
from rclpy.node import Node
import moveit_commander
from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import JointState
from agx_arm_msgs.msg import MoveMITMsg
from tf_transformations import quaternion_from_euler
import time
import numpy as np
from typing import Tuple, Optional, List
import logging


class HybridController(Node):
    """混合控制器：MoveIt + MIT 模式"""
    
    def __init__(self, node_name: str = 'hybrid_controller'):
        super().__init__(node_name)
        
        self.logger = logging.getLogger('HybridController')
        
        self.logger.info('初始化混合控制器...')
        
        # 初始化 MoveIt
        moveit_commander.roscpp_initialize(sys.argv)
        
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.arm_group = moveit_commander.MoveGroupCommander('arm')
        
        # MoveIt 规划参数
        self.arm_group.set_max_velocity_scaling_factor(0.3)
        self.arm_group.set_max_acceleration_scaling_factor(0.3)
        self.arm_group.set_planning_time(10.0)
        self.arm_group.set_num_planning_attempts(10)
        
        # MIT 模式发布者
        self.mit_pub = self.create_publisher(
            MoveMITMsg,
            '/control/move_mit',
            10
        )
        
        # 夹爪控制发布者
        self.gripper_pub = self.create_publisher(
            JointState,
            '/control/joint_states',
            10
        )
        
        # 关节状态订阅者
        self.joint_states = None
        self.create_subscription(
            JointState,
            '/feedback/joint_states',
            self._joint_states_callback,
            10
        )
        
        # MIT 控制参数（基于官方推荐值）
        # 标准 Piper: 关节123 -> Kp=5.5 Kd=0.35, 关节456 -> Kp=10 Kd=0.8
        self.mit_params = {
            'compliant': {  # 柔顺模式（官方推荐值）
                'kp_123': 5.5,   # 关节 1-3
                'kd_123': 0.35,
                'kp_456': 10.0,  # 关节 4-6
                'kd_456': 0.8,
            },
            'stiff': {  # 刚性模式（快速运动）
                'kp_123': 10.0,
                'kd_123': 0.5,
                'kp_456': 20.0,
                'kd_456': 1.2,
            },
            'damping': {  # 阻尼模式（接触保护）
                'kp_123': 3.0,
                'kd_123': 0.3,
                'kp_456': 5.0,
                'kd_456': 0.6,
            }
        }
        
        # 期望速度控制（v_des）
        # False: v_des=0，阻尼模式，更稳定（推荐）
        # True: 需要规划速度，通过位置微分得到
        self.use_velocity_control = False
        
        # 控制频率（Hz）
        self.control_rate = 100
        
        self.logger.info('混合控制器初始化完成!')
        self.logger.info(f'机械臂规划组: {self.arm_group.get_name()}')
        self.logger.info(f'末端执行器: {self.arm_group.get_end_effector_link()}')
    
    def _joint_states_callback(self, msg: JointState):
        """关节状态回调"""
        self.joint_states = msg
    
    def get_current_joint_angles(self) -> Optional[List[float]]:
        """
        获取当前关节角度
        
        Returns:
            Optional[List[float]]: 关节角度列表，失败返回 None
        """
        if self.joint_states is None:
            return None
        
        # 提取机械臂关节（过滤夹爪）
        joint_names = [name for name in self.joint_states.name if 'joint' in name and 'gripper' not in name]
        joint_angles = []
        
        for name in sorted(joint_names):
            try:
                idx = self.joint_states.name.index(name)
                joint_angles.append(self.joint_states.position[idx])
            except ValueError:
                continue
        
        return joint_angles if len(joint_angles) > 0 else None
    
    def moveit_move_to_pose(self, 
                           x: float, y: float, z: float,
                           qx: float = 0.0, qy: float = 0.0, 
                           qz: float = 0.0, qw: float = 1.0,
                           wait: bool = True) -> bool:
        """
        使用 MoveIt 移动到目标位姿
        
        Args:
            x, y, z: 目标位置（米）
            qx, qy, qz, qw: 目标姿态四元数
            wait: 是否等待运动完成
            
        Returns:
            bool: 成功返回 True
        """
        self.logger.info(f'MoveIt 移动到位姿: ({x:.3f}, {y:.3f}, {z:.3f})')
        
        target_pose = Pose()
        target_pose.position.x = x
        target_pose.position.y = y
        target_pose.position.z = z
        target_pose.orientation.x = qx
        target_pose.orientation.y = qy
        target_pose.orientation.z = qz
        target_pose.orientation.w = qw
        
        self.arm_group.set_pose_target(target_pose)
        success = self.arm_group.go(wait=wait)
        self.arm_group.stop()
        self.arm_group.clear_pose_targets()
        
        if success:
            self.logger.info('✓ MoveIt 运动完成')
        else:
            self.logger.error('✗ MoveIt 运动失败')
        
        return success
    
    def moveit_move_to_joint(self, joint_values: List[float], wait: bool = True) -> bool:
        """
        使用 MoveIt 移动到目标关节角度
        
        Args:
            joint_values: 目标关节角度列表（弧度）
            wait: 是否等待运动完成
            
        Returns:
            bool: 成功返回 True
        """
        self.logger.info(f'MoveIt 移动到关节位置: {[f"{j:.3f}" for j in joint_values]}')
        
        self.arm_group.set_joint_value_target(joint_values)
        success = self.arm_group.go(wait=wait)
        self.arm_group.stop()
        
        if success:
            self.logger.info('✓ MoveIt 运动完成')
        else:
            self.logger.error('✗ MoveIt 运动失败')
        
        return success
    
    def mit_control_joint(self,
                         joint_index: int,
                         p_des: float,
                         v_des: float = 0.0,
                         mode: str = 'compliant',
                         t_ff: float = 0.0):
        """
        使用 MIT 模式控制单个关节
        
        Args:
            joint_index: 关节索引 (1-6)
            p_des: 期望位置（弧度）
            v_des: 期望速度（弧度/秒）
            mode: 控制模式 ('compliant', 'stiff', 'damping')
            t_ff: 前馈力矩（N·m）
        """
        if mode not in self.mit_params:
            self.logger.error(f'未知的 MIT 控制模式: {mode}')
            return
        
        params = self.mit_params[mode]
        
        # 根据关节索引选择参数（关节1-3 和 4-6 使用不同参数）
        if 1 <= joint_index <= 3:
            kp = params['kp_123']
            kd = params['kd_123']
        elif 4 <= joint_index <= 6:
            kp = params['kp_456']
            kd = params['kd_456']
        else:
            self.logger.error(f'无效的关节索引: {joint_index}')
            return
        
        # v_des 默认为 0（阻尼模式，更稳定）
        if not self.use_velocity_control:
            v_des = 0.0
        
        msg = MoveMITMsg()
        msg.joint_index = [joint_index]
        msg.p_des = [p_des]
        msg.v_des = [v_des]
        msg.kp = [kp]
        msg.kd = [kd]
        msg.torque = [t_ff]
        
        self.mit_pub.publish(msg)
    
    def mit_control_all_joints(self,
                              joint_positions: List[float],
                              mode: str = 'compliant',
                              joint_velocities: Optional[List[float]] = None):
        """
        使用 MIT 模式控制所有关节
        
        Args:
            joint_positions: 期望关节位置列表（弧度）
            mode: 控制模式
            joint_velocities: 期望关节速度列表（弧度/秒），None 则全为 0
        """
        if mode not in self.mit_params:
            self.logger.error(f'未知的 MIT 控制模式: {mode}')
            return
        
        params = self.mit_params[mode]
        num_joints = len(joint_positions)
        
        # v_des 默认为 0（阻尼模式）
        if joint_velocities is None or not self.use_velocity_control:
            joint_velocities = [0.0] * num_joints
        
        # 为不同关节分配不同的 kp, kd
        kp_list = []
        kd_list = []
        for i in range(num_joints):
            joint_idx = i + 1  # 关节索引从 1 开始
            if 1 <= joint_idx <= 3:
                kp_list.append(params['kp_123'])
                kd_list.append(params['kd_123'])
            elif 4 <= joint_idx <= 6:
                kp_list.append(params['kp_456'])
                kd_list.append(params['kd_456'])
        
        msg = MoveMITMsg()
        msg.joint_index = list(range(1, num_joints + 1))
        msg.p_des = joint_positions
        msg.v_des = joint_velocities
        msg.kp = kp_list
        msg.kd = kd_list
        msg.torque = [0.0] * num_joints
        
        self.mit_pub.publish(msg)
    
    def mit_fine_tune_position(self,
                              target_joint_positions: List[float],
                              duration: float = 3.0,
                              tolerance: float = 0.01,
                              mode: str = 'compliant') -> bool:
        """
        使用 MIT 模式精细调整位置（柔顺控制）
        
        Args:
            target_joint_positions: 目标关节位置
            duration: 最大持续时间（秒）
            tolerance: 位置容差（弧度）
            mode: 控制模式
            
        Returns:
            bool: 成功到达返回 True
        """
        self.logger.info(f'MIT 精细调整: 目标={[f"{j:.3f}" for j in target_joint_positions]}')
        
        start_time = time.time()
        rate = self.create_rate(self.control_rate)
        
        while rclpy.ok():
            elapsed = time.time() - start_time
            
            if elapsed > duration:
                self.logger.warning('MIT 精细调整超时')
                return False
            
            # 发送 MIT 控制指令
            self.mit_control_all_joints(target_joint_positions, mode=mode)
            
            # 检查是否到达目标
            current_joints = self.get_current_joint_angles()
            if current_joints is not None:
                errors = [abs(c - t) for c, t in zip(current_joints, target_joint_positions)]
                max_error = max(errors)
                
                if max_error < tolerance:
                    self.logger.info(f'✓ MIT 精细调整完成，误差={max_error:.4f} rad')
                    return True
            
            rate.sleep()
        
        return False
    
    def compliant_move_to_position(self,
                                  target_position: Tuple[float, float, float],
                                  target_orientation: Tuple[float, float, float, float],
                                  compliance_distance: float = 0.05) -> bool:
        """
        柔顺运动到目标位置（MoveIt 接近 + MIT 柔顺精调）
        
        Args:
            target_position: 目标位置 (x, y, z)
            target_orientation: 目标姿态四元数 (qx, qy, qz, qw)
            compliance_distance: 切换到 MIT 模式的距离（米）
            
        Returns:
            bool: 成功返回 True
        """
        x, y, z = target_position
        qx, qy, qz, qw = target_orientation
        
        # 阶段 1: 使用 MoveIt 移动到接近位置
        approach_z = z + compliance_distance
        self.logger.info(f'阶段 1: MoveIt 接近位置 ({x:.3f}, {y:.3f}, {approach_z:.3f})')
        
        success = self.moveit_move_to_pose(x, y, approach_z, qx, qy, qz, qw)
        if not success:
            self.logger.error('MoveIt 接近失败')
            return False
        
        time.sleep(0.5)  # 等待稳定
        
        # 阶段 2: 切换到 MIT 模式进行柔顺下降
        self.logger.info(f'阶段 2: MIT 柔顺下降到目标 ({x:.3f}, {y:.3f}, {z:.3f})')
        
        # 计算目标关节角度（使用逆运动学）
        target_pose = Pose()
        target_pose.position.x = x
        target_pose.position.y = y
        target_pose.position.z = z
        target_pose.orientation.x = qx
        target_pose.orientation.y = qy
        target_pose.orientation.z = qz
        target_pose.orientation.w = qw
        
        self.arm_group.set_pose_target(target_pose)
        plan = self.arm_group.plan()
        self.arm_group.clear_pose_targets()
        
        # 检查规划是否成功（ROS2 Humble/Jazzy 返回元组）
        if isinstance(plan, tuple):
            success_flag, trajectory, planning_time, error_code = plan
        else:
            success_flag = plan
            trajectory = plan
        
        if not success_flag:
            self.logger.error('无法计算目标关节角度')
            return False
        
        # 获取目标关节位置
        if hasattr(trajectory, 'joint_trajectory') and len(trajectory.joint_trajectory.points) > 0:
            target_joints = list(trajectory.joint_trajectory.points[-1].positions)
        else:
            self.logger.error('轨迹为空')
            return False
        
        # 使用 MIT 模式精细调整
        success = self.mit_fine_tune_position(
            target_joints, 
            duration=5.0,
            tolerance=0.02,
            mode='compliant'
        )
        
        return success
    
    def set_gripper(self, width: float, force: float = 1.0, wait_time: float = 1.0):
        """
        控制夹爪
        
        Args:
            width: 夹爪开口宽度（米）
            force: 夹持力（牛顿）
            wait_time: 等待时间（秒）
        """
        self.logger.info(f'设置夹爪: 宽度={width:.3f}m, 力={force:.1f}N')
        
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['gripper']
        msg.position = [width]
        msg.effort = [force]
        
        self.gripper_pub.publish(msg)
        time.sleep(wait_time)
    
    def open_gripper(self, width: float = 0.08):
        """打开夹爪"""
        self.set_gripper(width, force=1.0)
    
    def close_gripper(self, width: float = 0.03, force: float = 1.5):
        """关闭夹爪（抓取）"""
        self.set_gripper(width, force=force)
    
    def go_home(self) -> bool:
        """回到零位"""
        self.logger.info('返回零位')
        self.arm_group.set_named_target('home')
        success = self.arm_group.go(wait=True)
        self.arm_group.stop()
        self.arm_group.clear_pose_targets()
        return success
    
    def execute_grasp_sequence(self,
                              approach_pose: Tuple[Tuple[float, float, float], Tuple[float, float, float, float]],
                              grasp_pose: Tuple[Tuple[float, float, float], Tuple[float, float, float, float]],
                              retreat_pose: Tuple[Tuple[float, float, float], Tuple[float, float, float, float]],
                              pre_grasp_width: float = 0.08,
                              grasp_width: float = 0.03,
                              grasp_force: float = 2.0) -> bool:
        """
        执行完整的抓取序列
        
        Args:
            approach_pose: 接近位姿 ((x, y, z), (qx, qy, qz, qw))
            grasp_pose: 抓取位姿
            retreat_pose: 撤退位姿
            pre_grasp_width: 预抓取夹爪宽度
            grasp_width: 抓取夹爪宽度
            grasp_force: 抓取力
            
        Returns:
            bool: 成功返回 True
        """
        self.logger.info('='*60)
        self.logger.info('开始执行抓取序列')
        self.logger.info('='*60)
        
        # 1. 打开夹爪
        self.logger.info('步骤 1: 打开夹爪')
        self.open_gripper(pre_grasp_width)
        time.sleep(0.5)
        
        # 2. 移动到接近位置（MoveIt）
        self.logger.info('步骤 2: 移动到接近位置')
        approach_pos, approach_ori = approach_pose
        success = self.moveit_move_to_pose(*approach_pos, *approach_ori)
        if not success:
            self.logger.error('移动到接近位置失败')
            return False
        time.sleep(0.5)
        
        # 3. 柔顺下降到抓取位置（MIT）
        self.logger.info('步骤 3: 柔顺下降到抓取位置')
        grasp_pos, grasp_ori = grasp_pose
        success = self.compliant_move_to_position(grasp_pos, grasp_ori, compliance_distance=0.05)
        if not success:
            self.logger.warning('柔顺下降未完全到达，尝试继续')
        time.sleep(0.5)
        
        # 4. 闭合夹爪抓取
        self.logger.info('步骤 4: 闭合夹爪抓取')
        self.close_gripper(grasp_width, grasp_force)
        time.sleep(1.0)
        
        # 5. 撤退（MoveIt）
        self.logger.info('步骤 5: 撤退')
        retreat_pos, retreat_ori = retreat_pose
        success = self.moveit_move_to_pose(*retreat_pos, *retreat_ori)
        if not success:
            self.logger.error('撤退失败')
            return False
        
        self.logger.info('='*60)
        self.logger.info('✓ 抓取序列完成!')
        self.logger.info('='*60)
        
        return True
    
    def set_mit_params(self, mode: str, kp_123: float, kd_123: float, 
                      kp_456: float, kd_456: float):
        """
        设置 MIT 控制参数
        
        Args:
            mode: 模式名称
            kp_123: 关节1-3的位置增益
            kd_123: 关节1-3的速度增益
            kp_456: 关节4-6的位置增益
            kd_456: 关节4-6的速度增益
        """
        if mode in self.mit_params:
            self.mit_params[mode]['kp_123'] = kp_123
            self.mit_params[mode]['kd_123'] = kd_123
            self.mit_params[mode]['kp_456'] = kp_456
            self.mit_params[mode]['kd_456'] = kd_456
            self.logger.info(f'更新 MIT 参数 [{mode}]: '
                           f'kp_123={kp_123}, kd_123={kd_123}, '
                           f'kp_456={kp_456}, kd_456={kd_456}')
        else:
            self.mit_params[mode] = {
                'kp_123': kp_123, 'kd_123': kd_123,
                'kp_456': kp_456, 'kd_456': kd_456
            }
            self.logger.info(f'创建新 MIT 参数模式 [{mode}]')
    
    def shutdown(self):
        """清理资源"""
        moveit_commander.roscpp_shutdown()


def test_hybrid_controller():
    """测试混合控制器"""
    import logging
    
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )
    
    rclpy.init()
    
    try:
        controller = HybridController()
        
        print("\n" + "="*60)
        print("混合控制器测试")
        print("="*60)
        print("确保已启动机械臂控制节点:")
        print("ros2 launch agx_arm_ctrl start_single_agx_arm_moveit.launch.py \\")
        print("  can_port:=can0 arm_type:=piper effector_type:=agx_gripper")
        print("="*60)
        
        input("\n按 Enter 键开始测试...")
        
        # 测试 1: 回零位
        print("\n测试 1: 回零位")
        controller.go_home()
        time.sleep(1.0)
        
        # 测试 2: MoveIt 移动
        print("\n测试 2: MoveIt 移动到测试位置")
        controller.moveit_move_to_joint([0.0, 0.4, -0.6, 0.0, 0.2, 0.0])
        time.sleep(1.0)
        
        # 测试 3: MIT 精细调整
        print("\n测试 3: MIT 精细调整")
        target_joints = [0.0, 0.45, -0.65, 0.0, 0.2, 0.0]
        controller.mit_fine_tune_position(target_joints, duration=3.0, mode='compliant')
        time.sleep(1.0)
        
        # 测试 4: 夹爪控制
        print("\n测试 4: 夹爪测试")
        controller.open_gripper(0.08)
        time.sleep(1.0)
        controller.close_gripper(0.03, 1.5)
        time.sleep(1.0)
        controller.open_gripper(0.08)
        
        # 回零位
        print("\n返回零位")
        controller.go_home()
        
        print("\n✓ 测试完成!")
        
    except KeyboardInterrupt:
        print("\n用户中断")
    except Exception as e:
        print(f"\n✗ 发生错误: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if 'controller' in locals():
            controller.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    test_hybrid_controller()

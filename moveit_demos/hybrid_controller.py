#!/usr/bin/env python3
"""
纯 MoveIt 控制模块（ROS 2 原生接口版）
功能：MoveIt 2 位姿规划 + 夹爪控制
适配 Piper 机械臂 + AGX Gripper
"""

import sys
import time
import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup

# MoveIt 2 消息
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest,
    Constraints,
    JointConstraint,
    PositionConstraint,
    OrientationConstraint,
    BoundingVolume,
    PlanningOptions,
    RobotState,
    MoveItErrorCodes,
)
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
from std_msgs.msg import Header
from sensor_msgs.msg import JointState

# 夹爪消息
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory

from typing import Tuple, Optional, List


def euler_to_quaternion(roll: float, pitch: float, yaw: float):
    """欧拉角 → 四元数"""
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


class HybridController(Node):
    """纯 MoveIt 控制器（类名保留以便兼容原调用）"""

    def __init__(self, node_name: str = 'hybrid_controller'):
        super().__init__(node_name)

        self.get_logger().info('初始化 MoveIt 控制器...')

        # ========== MoveIt 2 配置 ==========
        self.planning_group = 'arm'
        self.robot_base_frame = 'base_link'
        self.robot_tip_frame = 'link6'
        self.arm_joint_names = [
            'joint1', 'joint2', 'joint3',
            'joint4', 'joint5', 'joint6',
        ]
        self.home_joints = [0.0] * 6

        # ========== 姿态约束调试开关 ==========
        self.use_orientation_constraint = True
        self.orientation_tolerance = 0.2   # 弧度
        self.position_tolerance = 0.01     # 米

        # MoveGroup Action 客户端
        self.move_group_client = ActionClient(
            self,
            MoveGroup,
            '/move_action',
            callback_group=ReentrantCallbackGroup(),
        )
        if not self.move_group_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('✗ 无法连接到 /move_action，请确认 MoveIt 已启动')
            raise RuntimeError('MoveGroup Action 不可用')
        self.get_logger().info('✓ 已连接 MoveGroup Action')

        # ========== 关节状态订阅 ==========
        self.joint_states: Optional[JointState] = None
        self.create_subscription(
            JointState,
            '/feedback/joint_states',
            self._joint_states_callback,
            10,
        )

        # ========== 夹爪控制（双通道） ==========
        self.gripper_traj_pub = self.create_publisher(
            JointTrajectory,
            '/gripper_controller/joint_trajectory',
            10,
        )
        self.gripper_action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/gripper_controller/follow_joint_trajectory',
            callback_group=ReentrantCallbackGroup(),
        )
        self._gripper_action_available = self.gripper_action_client.wait_for_server(
            timeout_sec=1.0
        )
        if self._gripper_action_available:
            self.get_logger().info('✓ 夹爪 Action 可用')
        else:
            self.get_logger().info('ℹ 夹爪 Action 不可用，将使用 Trajectory 话题')

        self.gripper_joint_name = 'gripper'

        self.get_logger().info('✓ 控制器初始化完成')

    # ==================================================================
    # 关节状态
    # ==================================================================
    def _joint_states_callback(self, msg: JointState):
        self.joint_states = msg

    def get_current_joint_angles(self) -> Optional[List[float]]:
        if self.joint_states is None:
            return None
        joint_angles = []
        for name in self.arm_joint_names:
            if name in self.joint_states.name:
                idx = self.joint_states.name.index(name)
                joint_angles.append(self.joint_states.position[idx])
            else:
                return None
        return joint_angles if len(joint_angles) == 6 else None

    # ==================================================================
    # MoveGroup Action 底层封装
    # ==================================================================
    def _base_request(self) -> MotionPlanRequest:
        req = MotionPlanRequest()
        req.group_name = self.planning_group
        req.allowed_planning_time = 10.0
        req.max_velocity_scaling_factor = 0.3
        req.max_acceleration_scaling_factor = 0.3
        req.num_planning_attempts = 10
        req.start_state = RobotState()
        req.start_state.is_diff = True
        return req

    def _make_planning_options(self, plan_only: bool) -> PlanningOptions:
        opts = PlanningOptions()
        opts.plan_only = plan_only
        opts.look_around = False
        opts.replan = True
        opts.replan_delay = 1.0
        return opts

    def _make_joint_constraints(self, joint_values: List[float]) -> Constraints:
        c = Constraints()
        for name, value in zip(self.arm_joint_names, joint_values):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = float(value)
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            c.joint_constraints.append(jc)
        return c

    def _make_pose_constraints(self,
                               x: float, y: float, z: float,
                               qx: float, qy: float, qz: float, qw: float
                               ) -> Constraints:
        c = Constraints()

        # ---- 位置约束 ----
        pc = PositionConstraint()
        pc.header = Header()
        pc.header.frame_id = self.robot_base_frame
        pc.link_name = self.robot_tip_frame
        pc.target_point_offset.x = 0.0
        pc.target_point_offset.y = 0.0
        pc.target_point_offset.z = 0.0
        pc.constraint_region = BoundingVolume()
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [self.position_tolerance]
        pc.constraint_region.primitives.append(sphere)
        sp = Pose()
        sp.position.x = x
        sp.position.y = y
        sp.position.z = z
        sp.orientation.w = 1.0
        pc.constraint_region.primitive_poses.append(sp)
        pc.weight = 1.0
        c.position_constraints.append(pc)

        # ---- 姿态约束（可由开关关闭） ----
        if self.use_orientation_constraint:
            oc = OrientationConstraint()
            oc.header = Header()
            oc.header.frame_id = self.robot_base_frame
            oc.link_name = self.robot_tip_frame
            oc.orientation.x = qx
            oc.orientation.y = qy
            oc.orientation.z = qz
            oc.orientation.w = qw
            oc.absolute_x_axis_tolerance = self.orientation_tolerance
            oc.absolute_y_axis_tolerance = self.orientation_tolerance
            oc.absolute_z_axis_tolerance = self.orientation_tolerance
            oc.weight = 1.0
            c.orientation_constraints.append(oc)

        return c

    def _send_move_group(self, goal: MoveGroup.Goal,
                         wait_for_result: bool = True,
                         timeout: float = 20.0):
        future = self.move_group_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        if not future.done():
            self.get_logger().error('MoveGroup 请求发送超时')
            return None
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('MoveGroup 请求被拒绝')
            return None
        if not wait_for_result:
            return goal_handle

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=timeout)
        if not result_future.done():
            self.get_logger().error('MoveGroup 执行超时')
            return None

        result = result_future.result().result

        # ---- 错误码诊断输出 ----
        if result.error_code.val != MoveItErrorCodes.SUCCESS:
            self.get_logger().error(
                f'MoveIt 错误码: {result.error_code.val} '
                f'(SUCCESS=1, FAILURE=99999, PLANNING_FAILED=-1, '
                f'GOAL_IN_COLLISION=-12, GOAL_CONSTRAINTS_VIOLATED=-14, '
                f'NO_IK_SOLUTION=-31)'
            )
        return result

    @staticmethod
    def _is_success(result) -> bool:
        if result is None:
            return False
        return result.error_code.val == MoveItErrorCodes.SUCCESS

    # ==================================================================
    # MoveIt 高层接口
    # ==================================================================
    def moveit_move_to_joint(self, joint_values: List[float],
                             wait: bool = True) -> bool:
        self.get_logger().info(
            f'MoveIt 移动到关节: {[f"{j:.3f}" for j in joint_values]}'
        )
        goal = MoveGroup.Goal()
        goal.request = self._base_request()
        goal.request.goal_constraints.append(
            self._make_joint_constraints(joint_values)
        )
        goal.planning_options = self._make_planning_options(plan_only=False)

        result = self._send_move_group(goal, wait_for_result=wait,
                                       timeout=20.0)
        if not wait:
            return True
        ok = self._is_success(result)
        self.get_logger().info('✓ MoveIt 关节运动完成' if ok else '✗ MoveIt 关节运动失败')
        return ok

    def moveit_move_to_pose(self,
                            x: float, y: float, z: float,
                            qx: float = 0.0, qy: float = 0.0,
                            qz: float = 0.0, qw: float = 1.0,
                            wait: bool = True) -> bool:
        self.get_logger().info(
            f'MoveIt 移动到位姿: ({x:.3f}, {y:.3f}, {z:.3f}) '
            f'姿态=({qx:.3f}, {qy:.3f}, {qz:.3f}, {qw:.3f}) '
            f'[姿态约束={"开" if self.use_orientation_constraint else "关"}]'
        )
        goal = MoveGroup.Goal()
        goal.request = self._base_request()
        goal.request.goal_constraints.append(
            self._make_pose_constraints(x, y, z, qx, qy, qz, qw)
        )
        goal.planning_options = self._make_planning_options(plan_only=False)

        result = self._send_move_group(goal, wait_for_result=wait,
                                       timeout=20.0)
        if not wait:
            return True
        ok = self._is_success(result)
        self.get_logger().info('✓ MoveIt 位姿运动完成' if ok else '✗ MoveIt 位姿运动失败')
        return ok

    def plan_to_pose(self,
                     x: float, y: float, z: float,
                     qx: float, qy: float, qz: float, qw: float
                     ) -> Optional[List[float]]:
        """只规划，不执行；返回规划轨迹的最终关节角度"""
        goal = MoveGroup.Goal()
        goal.request = self._base_request()
        goal.request.goal_constraints.append(
            self._make_pose_constraints(x, y, z, qx, qy, qz, qw)
        )
        goal.planning_options = self._make_planning_options(plan_only=True)

        result = self._send_move_group(goal, wait_for_result=True, timeout=10.0)
        if not self._is_success(result):
            self.get_logger().error('规划失败')
            return None

        traj = result.planned_trajectory
        if traj is None or len(traj.joint_trajectory.points) == 0:
            self.get_logger().error('规划轨迹为空')
            return None
        return list(traj.joint_trajectory.points[-1].positions)

    def go_home(self) -> bool:
        """回到零位（所有关节为 0）"""
        self.get_logger().info('返回零位')
        return self.moveit_move_to_joint(self.home_joints)

    # ==================================================================
    # 夹爪控制（双通道）
    # ==================================================================
    def _send_gripper_trajectory(self, width: float,
                                 duration: float = 1.0) -> bool:
        try:
            traj = JointTrajectory()
            traj.header.stamp = self.get_clock().now().to_msg()
            traj.joint_names = [self.gripper_joint_name]

            point = JointTrajectoryPoint()
            point.positions = [float(width)]
            point.velocities = [0.1]
            point.accelerations = [0.1]
            point.time_from_start.sec = int(duration)
            point.time_from_start.nanosec = int((duration % 1) * 1e9)
            traj.points.append(point)

            self.gripper_traj_pub.publish(traj)
            self.get_logger().info(
                f'✓ Trajectory 已发送: joint={self.gripper_joint_name}, '
                f'width={width:.3f}'
            )
            return True
        except Exception as e:
            self.get_logger().error(f'✗ Trajectory 发送失败: {e}')
            return False

    def _send_gripper_action(self, width: float,
                             duration: float = 1.0) -> bool:
        try:
            goal = FollowJointTrajectory.Goal()
            goal.trajectory = JointTrajectory()
            goal.trajectory.header.stamp = self.get_clock().now().to_msg()
            goal.trajectory.joint_names = [self.gripper_joint_name]

            point = JointTrajectoryPoint()
            point.positions = [float(width)]
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
            rclpy.spin_until_future_complete(
                self, result_future, timeout_sec=duration + 2.0
            )
            return result_future.done()
        except Exception as e:
            self.get_logger().warning(f'Action 控制失败: {e}')
            return False

    def set_gripper(self, width: float, force: float = 1.0,
                    wait_time: float = 1.5) -> bool:
        self.get_logger().info(
            f'设置夹爪: 宽度={width:.3f}m 力={force:.1f}N'
        )
        if self._gripper_action_available and self._send_gripper_action(
                width, duration=1.0):
            time.sleep(wait_time)
            return True
        ok = self._send_gripper_trajectory(width, duration=1.0)
        time.sleep(wait_time)
        return ok

    def open_gripper(self, width: float = 0.08):
        return self.set_gripper(width, force=1.0)

    def close_gripper(self, width: float = 0.03, force: float = 1.5):
        return self.set_gripper(width, force=force)

    # ==================================================================
    # 抓取序列（全 MoveIt）
    # ==================================================================
    def execute_grasp_sequence(self,
                               approach_pose, grasp_pose, retreat_pose,
                               pre_grasp_width: float = 0.08,
                               grasp_width: float = 0.03,
                               grasp_force: float = 2.0) -> bool:
        self.get_logger().info('=' * 60)
        self.get_logger().info('开始抓取序列（纯 MoveIt）')
        self.get_logger().info('=' * 60)

        # 步骤 1: 打开夹爪
        self.get_logger().info('步骤 1: 打开夹爪')
        self.open_gripper(pre_grasp_width)
        time.sleep(0.5)

        # 步骤 2: 移动到接近位姿
        self.get_logger().info('步骤 2: 移动到接近位姿')
        apos, aori = approach_pose
        if not self.moveit_move_to_pose(*apos, *aori):
            return False
        time.sleep(0.5)

        # 步骤 3: MoveIt 下降到抓取位姿
        self.get_logger().info('步骤 3: MoveIt 下降到抓取位姿')
        gpos, gori = grasp_pose
        if not self.moveit_move_to_pose(*gpos, *gori):
            self.get_logger().error('抓取位姿规划失败')
            return False
        time.sleep(0.5)

        # 步骤 4: 闭合夹爪
        self.get_logger().info('步骤 4: 闭合夹爪')
        self.close_gripper(grasp_width, grasp_force)
        time.sleep(1.0)

        # 步骤 5: 撤退
        self.get_logger().info('步骤 5: 撤退')
        rpos, rori = retreat_pose
        if not self.moveit_move_to_pose(*rpos, *rori):
            return False

        self.get_logger().info('✓ 抓取序列完成')
        return True

    def shutdown(self):
        try:
            self.move_group_client.destroy()
        except Exception:
            pass
        try:
            self.gripper_action_client.destroy()
        except Exception:
            pass


# ======================================================================
# 测试入口
# ======================================================================
def test_moveit_controller():
    rclpy.init()
    controller = None
    try:
        controller = HybridController()
        print('\n' + '=' * 60)
        print('MoveIt 控制器测试')
        print('=' * 60)
        print('请确保已启动:')
        print('  ros2 launch agx_arm_ctrl start_single_agx_arm_moveit.launch.py \\')
        print('    can_port:=can0 arm_type:=piper effector_type:=agx_gripper')
        print('=' * 60)
        input('\n按 Enter 开始测试...')

        print('\n测试 1: 回零位')
        controller.go_home()
        time.sleep(1.0)

        print('\n测试 2: 只位置约束的位姿移动')
        controller.use_orientation_constraint = False
        ok = controller.moveit_move_to_pose(0.379, 0.100, 0.282,
                                             0.0, 1.0, 0.0, 0.0)
        print('只位置约束:', '✓ 成功' if ok else '✗ 失败')
        time.sleep(0.5)

        print('\n测试 3: 位置+姿态约束的位姿移动')
        controller.use_orientation_constraint = True
        ok = controller.moveit_move_to_pose(0.379, 0.100, 0.282,
                                             0.0, 1.0, 0.0, 0.0)
        print('位置+姿态:', '✓ 成功' if ok else '✗ 失败')

        print('\n返回零位')
        controller.go_home()
        print('\n✓ 测试完成!')

    except KeyboardInterrupt:
        print('\n用户中断')
    except Exception as e:
        print(f'\n✗ 错误: {e}')
        import traceback
        traceback.print_exc()
    finally:
        if controller is not None:
            controller.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    test_moveit_controller()
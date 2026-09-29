#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
基于视觉的蓝色纸巾抓取程序 (眼在手上 eye-in-hand)
- 机械臂：MoveIt MoveGroup action (/move_action) 只约束位置
- 夹爪：FollowJointTrajectory action
- 视觉：RealSense D435 + 蓝色检测
- 手眼：eye-in-hand，运行时用 TF 动态计算 T_base_camera
"""

import time
import numpy as np
import cv2
import pyrealsense2 as rs

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.time import Time
from rclpy.duration import Duration

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
from tf2_ros import Buffer, TransformListener

from scipy.spatial.transform import Rotation as R


# ============================================================
#                       配置区
# ============================================================
ARM_GROUP       = 'arm'
BASE_FRAME      = 'base_link'
TIP_FRAME       = 'tcp_link'      # MoveIt 规划用的末端 link
EFFECTOR_FRAME  = 'link6'         # 与手眼标定时保持一致

ARM_JOINT_NAMES = ['joint1', 'joint2', 'joint3',
                   'joint4', 'joint5', 'joint6']

JOINT_LIMITS = [
    (-2.6179938, 2.6179938),
    (0.0,       3.1415926),
    (-2.9670597, 0.0),
    (-1.7453292, 1.7453292),
    (-1.2217304, 1.2217304),
    (-2.0943951, 2.0943951),
]

# 夹爪
GRIPPER_JOINT_NAME   = 'gripper'
GRIPPER_OPEN_WIDTH   = 0.08
GRIPPER_CLOSE_WIDTH  = 0.02
GRIPPER_FORCE        = 1.5

# 抓取高度偏移
GRASP_HEIGHT_OFFSET   = 0.15
GRASP_APPROACH_OFFSET = 0.05

# 蓝色检测 HSV
BLUE_LOWER = np.array([90, 80, 80])
BLUE_UPPER = np.array([130, 255, 255])

# ---------- 手眼标定结果 (eye-in-hand) ----------
# 语义：把相机坐标系中的点转到 link6 坐标系 (T_link6_camera)
# 方向说明：
#   - 如果抓取位置完全不对（例如 z 是负数、偏离十几米），把下面这一行改成 False
#     表示标定结果其实是 T_camera_link6，代码会自动取逆
HAND_EYE_IS_EFFECTOR_TO_CAMERA = True

CAM_POS = np.array([
    -0.070701,
    -0.000912,
    0.037813
])
CAM_QUAT = np.array([
    -0.133285,
    0.126056,
    -0.659821,
    0.728684
])


class VisionGraspBlue(Node):
    def __init__(self):
        super().__init__('vision_grasp_blue')
        self.get_logger().info('=' * 60)
        self.get_logger().info('初始化视觉抓取节点 (eye-in-hand)')
        self.get_logger().info('=' * 60)

        # ---------- 手眼标定结果：常量 ----------
        T_raw = self.create_transform_matrix(CAM_POS, CAM_QUAT)
        if HAND_EYE_IS_EFFECTOR_TO_CAMERA:
            self.T_effector_camera = T_raw
        else:
            self.T_effector_camera = np.linalg.inv(T_raw)
        self.get_logger().info(
            f'T_effector_camera (常量):\n{self.T_effector_camera}'
        )

        # ---------- TF listener ----------
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ---------- 关节状态订阅 ----------
        self.current_joint_positions = []
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/control/joint_states',
            self.joint_state_callback,
            10
        )

        # ---------- MoveGroup Action ----------
        self.arm_action_client = ActionClient(
            self, MoveGroup, '/move_action',
            callback_group=ReentrantCallbackGroup()
        )
        if not self.arm_action_client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError('无法连接 /move_action，请确认 MoveIt 已启动')
        self.get_logger().info('✓ 已连接 MoveGroup action (/move_action)')

        # ---------- 夹爪 action ----------
        self.gripper_action_client = ActionClient(
            self, FollowJointTrajectory,
            '/gripper_controller/follow_joint_trajectory',
            callback_group=ReentrantCallbackGroup()
        )
        if not self.gripper_action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn(
                '⚠ 未找到夹爪 action，退化为话题发布 joint_trajectory'
            )
            self.gripper_action_client = None
            self.gripper_traj_pub = self.create_publisher(
                JointTrajectory,
                '/gripper_controller/joint_trajectory',
                10
            )
        else:
            self.gripper_traj_pub = None
            self.get_logger().info('✓ 已连接夹爪 FollowJointTrajectory action')

        # ---------- RealSense ----------
        self.pipeline = None
        self.align = None
        self.depth_scale = 0.001

        self.get_logger().info('✓ 初始化完成')

    # ============================================================
    #                 工具函数
    # ============================================================
    def create_transform_matrix(self, position, quaternion):
        T = np.eye(4)
        T[:3, :3] = R.from_quat(quaternion).as_matrix()
        T[:3, 3] = position
        return T

    def joint_state_callback(self, msg):
        self.current_joint_positions = list(msg.position)

    def get_current_joint_positions(self):
        return self.current_joint_positions if self.current_joint_positions else None

    # ============================================================
    #      眼在手上关键：动态查询 T_base_effector 并计算 T_base_camera
    # ============================================================
    def get_T_base_effector(self):
        """查询当前 base_link -> link6 的变换 (T_base_effector)"""
        for _ in range(30):
            rclpy.spin_once(self, timeout_sec=0.05)
            try:
                t = self.tf_buffer.lookup_transform(
                    BASE_FRAME, EFFECTOR_FRAME, Time(),
                    timeout=Duration(seconds=0.5)
                )
                pos = np.array([t.transform.translation.x,
                                t.transform.translation.y,
                                t.transform.translation.z])
                quat = np.array([t.transform.rotation.x,
                                 t.transform.rotation.y,
                                 t.transform.rotation.z,
                                 t.transform.rotation.w])
                return self.create_transform_matrix(pos, quat)
            except Exception as e:
                self.get_logger().warn(f'TF lookup failed: {e}')
        return None

    def get_T_base_camera(self):
        """
        T_base_camera = T_base_effector @ T_effector_camera
        每次调用都会重新查询 TF，保证反映机器人当前姿态
        """
        T_base_effector = self.get_T_base_effector()
        if T_base_effector is None:
            return None
        return T_base_effector @ self.T_effector_camera

    # ============================================================
    #                  MoveGroup action 封装
    # ============================================================
    def _send_move_group_goal(self, goal_msg, timeout_sec=20.0):
        send_future = self.arm_action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=10.0)
        if not send_future.done():
            self.get_logger().error('✗ 发送目标超时')
            return False

        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error('✗ MoveGroup 目标被拒绝')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=timeout_sec)
        if not result_future.done():
            self.get_logger().error('✗ 等待结果超时')
            return False

        result = result_future.result().result
        if hasattr(result, 'error_code') and result.error_code.val != 1:
            self.get_logger().error(
                f'✗ 运动失败, error_code={result.error_code.val}'
            )
            return False
        return True

    def _build_joint_constraints(self, joint_values):
        constraints = Constraints()
        for name, value in zip(ARM_JOINT_NAMES, joint_values):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = float(value)
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
        return constraints

    def _build_position_only_constraints(self, x, y, z, radius=0.03):
        constraints = Constraints()

        pc = PositionConstraint()
        pc.header = Header()
        pc.header.frame_id = BASE_FRAME
        pc.link_name = TIP_FRAME
        pc.target_point_offset.x = 0.0
        pc.target_point_offset.y = 0.0
        pc.target_point_offset.z = 0.0

        pc.constraint_region = BoundingVolume()
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [float(radius)]
        pc.constraint_region.primitives.append(sphere)

        sphere_pose = Pose()
        sphere_pose.position.x = float(x)
        sphere_pose.position.y = float(y)
        sphere_pose.position.z = float(z)
        pc.constraint_region.primitive_poses.append(sphere_pose)

        pc.weight = 1.0
        constraints.position_constraints.append(pc)
        return constraints

    def go_to_joint_position(self, joint_values, name='关节位置'):
        if len(joint_values) != 6:
            self.get_logger().error(f'✗ 需要 6 个关节值，得到 {len(joint_values)}')
            return False

        for i, (v, (lo, hi)) in enumerate(zip(joint_values, JOINT_LIMITS)):
            if v < lo - 0.01 or v > hi + 0.01:
                self.get_logger().error(
                    f'✗ joint{i+1}={v:.3f} 超出限制 [{lo:.3f}, {hi:.3f}]'
                )
                return False

        self.get_logger().info(f'移动到 {name}: {[f"{j:.3f}" for j in joint_values]}')

        goal_msg = MoveGroup.Goal()
        req = MotionPlanRequest()
        req.group_name = ARM_GROUP
        req.allowed_planning_time = 10.0
        req.max_velocity_scaling_factor = 0.2
        req.max_acceleration_scaling_factor = 0.2
        req.num_planning_attempts = 20
        req.start_state = RobotState()
        req.start_state.is_diff = True
        req.goal_constraints.append(self._build_joint_constraints(joint_values))

        opts = PlanningOptions()
        opts.plan_only = False
        opts.look_around = True
        opts.replan = True
        opts.replan_delay = 2.0

        goal_msg.request = req
        goal_msg.planning_options = opts
        return self._send_move_group_goal(goal_msg)

    def go_to_position(self, position, name='目标位置', radius=0.03):
        self.get_logger().info(
            f'移动到 {name}: '
            f'({position[0]:.3f}, {position[1]:.3f}, {position[2]:.3f}) '
            f'半径容差={radius:.3f} m'
        )

        goal_msg = MoveGroup.Goal()
        req = MotionPlanRequest()
        req.group_name = ARM_GROUP
        req.allowed_planning_time = 10.0
        req.max_velocity_scaling_factor = 0.2
        req.max_acceleration_scaling_factor = 0.2
        req.num_planning_attempts = 20
        req.start_state = RobotState()
        req.start_state.is_diff = True
        req.goal_constraints.append(
            self._build_position_only_constraints(
                position[0], position[1], position[2], radius
            )
        )

        opts = PlanningOptions()
        opts.plan_only = False
        opts.look_around = True
        opts.replan = True
        opts.replan_delay = 2.0

        goal_msg.request = req
        goal_msg.planning_options = opts
        return self._send_move_group_goal(goal_msg)

    def go_to_home(self):
        self.get_logger().info('返回零位...')
        return self.go_to_joint_position([0.0] * 6, '零位')

    # ============================================================
    #                 夹爪 action 封装
    # ============================================================
    def _send_gripper_trajectory(self, width, duration=1.0):
        traj = JointTrajectory()
        traj.header.stamp = self.get_clock().now().to_msg()
        traj.joint_names = [GRIPPER_JOINT_NAME]

        pt = JointTrajectoryPoint()
        pt.positions = [float(width)]
        pt.velocities = [0.1]
        pt.accelerations = [0.1]
        pt.time_from_start.sec = int(duration)
        pt.time_from_start.nanosec = int((duration % 1) * 1e9)
        traj.points.append(pt)

        if self.gripper_action_client is not None:
            goal = FollowJointTrajectory.Goal()
            goal.trajectory = traj

            send_future = self.gripper_action_client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, send_future, timeout_sec=3.0)
            if not send_future.done():
                self.get_logger().error('✗ 夹爪目标发送超时')
                return False

            goal_handle = send_future.result()
            if goal_handle is None or not goal_handle.accepted:
                self.get_logger().error('✗ 夹爪目标被拒绝')
                return False

            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(
                self, result_future, timeout_sec=duration + 3.0
            )
            if not result_future.done():
                self.get_logger().warn('⚠ 夹爪 action 结果超时')
                return False
            return True
        else:
            self.gripper_traj_pub.publish(traj)
            time.sleep(duration + 0.5)
            return True

    def open_gripper(self, width=GRIPPER_OPEN_WIDTH):
        self.get_logger().info(f'打开夹爪: {width:.3f} m')
        return self._send_gripper_trajectory(width, duration=1.0)

    def close_gripper(self, width=GRIPPER_CLOSE_WIDTH, force=GRIPPER_FORCE):
        self.get_logger().info(f'关闭夹爪: {width:.3f} m, force={force}')
        return self._send_gripper_trajectory(width, duration=1.2)

    # ============================================================
    #                 RealSense 部分
    # ============================================================
    def initialize_realsense(self):
        self.get_logger().info('初始化 RealSense D435...')
        try:
            self.pipeline = rs.pipeline()
            config = rs.config()
            config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
            config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

            profile = self.pipeline.start(config)
            self.align = rs.align(rs.stream.color)

            depth_sensor = profile.get_device().first_depth_sensor()
            self.depth_scale = depth_sensor.get_depth_scale()
            self.get_logger().info(f'✓ 相机启动, depth_scale={self.depth_scale}')

            for _ in range(30):
                self.pipeline.wait_for_frames()
            return True
        except Exception as e:
            self.get_logger().error(f'✗ RealSense 初始化失败: {e}')
            return False

    def get_camera_frames(self):
        try:
            frames = self.pipeline.wait_for_frames()
            aligned = self.align.process(frames)
            color_frame = aligned.get_color_frame()
            depth_frame = aligned.get_depth_frame()
            if not color_frame or not depth_frame:
                return None, None, None
            color = np.asanyarray(color_frame.get_data())
            depth = np.asanyarray(depth_frame.get_data())
            return color, depth, depth_frame
        except Exception as e:
            self.get_logger().error(f'获取帧失败: {e}')
            return None, None, None

    def detect_blue_object(self, color_image, depth_image, depth_frame):
        result = {
            'found': False,
            'center_pixel': None,
            'center_3d_camera': None,
            'center_3d_base': None,
            'contour_area': 0.0,
            'mask': None
        }

        hsv = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, BLUE_LOWER, BLUE_UPPER)

        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        result['mask'] = mask

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if len(contours) == 0:
            self.get_logger().warn('未检测到蓝色物体')
            return result

        largest = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest)
        if area < 500:
            self.get_logger().warn(f'蓝色区域太小: {area:.0f} px')
            return result

        M = cv2.moments(largest)
        if M['m00'] == 0:
            return result
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])

        result['center_pixel'] = (cx, cy)
        result['contour_area'] = area

        roi = depth_image[max(0, cy - 2):cy + 3, max(0, cx - 2):cx + 3]
        valid = roi[roi > 0]
        if len(valid) == 0:
            self.get_logger().warn(f'像素 ({cx},{cy}) 附近无有效深度')
            return result
        depth = float(np.median(valid)) * self.depth_scale

        intrinsics = depth_frame.profile.as_video_stream_profile().intrinsics
        p_cam = rs.rs2_deproject_pixel_to_point(intrinsics, [cx, cy], depth)
        result['center_3d_camera'] = np.array(p_cam)

        # ===== 眼在手上关键：动态获取 T_base_camera =====
        T_base_camera = self.get_T_base_camera()
        if T_base_camera is None:
            self.get_logger().error('✗ 无法获取 T_base_camera (TF 不可用)')
            return result

        p_cam_h = np.array([p_cam[0], p_cam[1], p_cam[2], 1.0])
        p_base = T_base_camera @ p_cam_h
        result['center_3d_base'] = p_base[:3]
        result['found'] = True

        self.get_logger().info('检测到蓝色物体:')
        self.get_logger().info(f'  像素: ({cx}, {cy})  深度: {depth:.3f} m')
        self.get_logger().info(
            f'  基座: ({p_base[0]:.3f}, {p_base[1]:.3f}, {p_base[2]:.3f})'
        )
        return result

    def visualize_detection(self, color_image, detection_result):
        vis = color_image.copy()
        if detection_result['found']:
            cx, cy = detection_result['center_pixel']
            cv2.circle(vis, (cx, cy), 5, (0, 255, 0), -1)
            cv2.line(vis, (cx - 20, cy), (cx + 20, cy), (0, 255, 0), 2)
            cv2.line(vis, (cx, cy - 20), (cx, cy + 20), (0, 255, 0), 2)
            b = detection_result['center_3d_base']
            cv2.putText(vis, f"({b[0]:.3f},{b[1]:.3f},{b[2]:.3f})",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, (0, 255, 0), 2)
        try:
            cv2.imshow('Blue Detection', vis)
            if detection_result['mask'] is not None:
                cv2.imshow('Mask', detection_result['mask'])
            cv2.waitKey(1)
        except cv2.error:
            pass

    # ============================================================
    #                     抓取序列
    # ============================================================
    def grasp_object_at_position(self, target_pos):
        self.get_logger().info('=' * 60)
        self.get_logger().info('开始抓取序列')
        self.get_logger().info('=' * 60)

        self.get_logger().info('步骤 1: 打开夹爪')
        if not self.open_gripper():
            return False
        time.sleep(0.5)

        approach_pos = np.array(target_pos, dtype=float).copy()
        approach_pos[2] += GRASP_HEIGHT_OFFSET
        self.get_logger().info(
            f'步骤 2: 预备位置 → ({approach_pos[0]:.3f}, '
            f'{approach_pos[1]:.3f}, {approach_pos[2]:.3f})'
        )
        if not self.go_to_position(approach_pos, '预备位置', radius=0.03):
            self.get_logger().error('✗ 预备位置失败')
            return False
        time.sleep(0.5)

        grasp_pos = np.array(target_pos, dtype=float).copy()
        grasp_pos[2] += GRASP_APPROACH_OFFSET
        self.get_logger().info(
            f'步骤 3: 抓取位置 → ({grasp_pos[0]:.3f}, '
            f'{grasp_pos[1]:.3f}, {grasp_pos[2]:.3f})'
        )
        if not self.go_to_position(grasp_pos, '抓取位置', radius=0.02):
            self.get_logger().error('✗ 下降到抓取位置失败')
            return False
        time.sleep(0.5)

        self.get_logger().info('步骤 4: 关闭夹爪')
        self.close_gripper(GRIPPER_CLOSE_WIDTH, GRIPPER_FORCE)
        time.sleep(1.5)

        self.get_logger().info('步骤 5: 抬起物体')
        self.go_to_position(approach_pos, '抬起位置', radius=0.03)
        time.sleep(0.5)

        place_pos = np.array([0.25, -0.15, 0.20])
        self.get_logger().info(
            f'步骤 6: 放置位置 → ({place_pos[0]:.3f}, '
            f'{place_pos[1]:.3f}, {place_pos[2]:.3f})'
        )
        if not self.go_to_position(place_pos, '放置位置', radius=0.03):
            self.get_logger().warn('⚠ 移动到放置位置失败，将在当前位置释放')
        time.sleep(0.5)

        self.get_logger().info('步骤 7: 释放物体')
        self.open_gripper()
        time.sleep(1.0)

        self.get_logger().info('步骤 8: 回零位')
        self.go_to_home()

        self.get_logger().info('✓ 抓取序列完成!')
        return True

    # ============================================================
    #                     主流程
    # ============================================================
    def run_vision_grasp(self):
        self.get_logger().info('=' * 60)
        self.get_logger().info('启动视觉抓取流程 (eye-in-hand)')
        self.get_logger().info('=' * 60)

        if not self.initialize_realsense():
            self.get_logger().error('✗ 相机初始化失败')
            return False

        self.get_logger().info('回零位...')
        self.go_to_home()
        time.sleep(1.0)
        # 回零位后，spin 几次让 TF 追上
        for _ in range(20):
            rclpy.spin_once(self, timeout_sec=0.05)

        max_attempts = 50
        for i in range(max_attempts):
            color, depth, depth_frame = self.get_camera_frames()
            if color is None:
                time.sleep(0.1)
                continue

            result = self.detect_blue_object(color, depth, depth_frame)
            self.visualize_detection(color, result)

            if result['found']:
                self.get_logger().info(
                    f'✓ 检测成功 (第 {i+1}/{max_attempts} 次)'
                )
                return self.grasp_object_at_position(result['center_3d_base'])

            time.sleep(0.1)

        self.get_logger().warn(f'⚠ {max_attempts} 次尝试后未检测到蓝色物体')
        return False

    def shutdown(self):
        try:
            if self.pipeline:
                self.pipeline.stop()
        except Exception:
            pass
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass


def main():
    rclpy.init()
    node = None
    try:
        node = VisionGraspBlue()

        print('\n' + '=' * 60)
        print('视觉抓取蓝色纸巾程序 (eye-in-hand)')
        print('=' * 60)
        print('请确认已启动:')
        print('  1. 机械臂 + MoveIt 节点 (提供 /move_action)')
        print('  2. 夹爪控制器 action')
        print('  3. RealSense D435 已连接')
        print('  4. TF 中 base_link → link6 可用 (机器人驱动在跑)')
        print('=' * 60)

        input('\n按 Enter 键开始视觉抓取...')

        success = node.run_vision_grasp()
        if success:
            print('\n✓ 任务完成!')
        else:
            print('\n✗ 任务失败')

    except KeyboardInterrupt:
        print('\n用户中断')
    except Exception as e:
        print(f'\n✗ 错误: {e}')
        import traceback
        traceback.print_exc()
    finally:
        if node is not None:
            node.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
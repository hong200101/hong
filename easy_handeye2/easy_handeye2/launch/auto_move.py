#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.time import Time

import numpy as np

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints, PositionConstraint, OrientationConstraint,
    BoundingVolume, MotionPlanRequest
)
from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive

from tf2_ros import Buffer, TransformListener
import tf_transformations


class AutoMover(Node):
    def __init__(self):
        super().__init__('auto_mover')

        # ========== 按你的机器人修改 ==========
        self.group_name = 'arm'          # MoveIt planning group 名
        self.ee_link    = 'link6'        # 末端 link
        self.base_frame = 'base_link'    # 基座 frame
        # ====================================

        self._action_client = ActionClient(self, MoveGroup, '/move_action')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('/move_action not available，请确认 move_group 已启动！')

    # ---------- 从 TF 查询当前末端位姿 ----------
    def current_pose(self):
        for _ in range(50):
            rclpy.spin_once(self, timeout_sec=0.1)
            try:
                t = self.tf_buffer.lookup_transform(
                    self.base_frame, self.ee_link, Time(),
                    timeout=Duration(seconds=0.5))
                p = Pose()
                p.position.x = t.transform.translation.x
                p.position.y = t.transform.translation.y
                p.position.z = t.transform.translation.z
                p.orientation = t.transform.rotation
                return p
            except Exception as e:
                self.get_logger().warn(f'TF lookup failed: {e}')
        return None

    # ---------- 用 /move_action 让机械臂走到目标位姿 ----------
    def goto_pose(self, pose: Pose):
        goal = MoveGroup.Goal()
        req = MotionPlanRequest()
        req.group_name = self.group_name
        req.num_planning_attempts = 10
        req.allowed_planning_time = 5.0
        req.max_velocity_scaling_factor = 0.3
        req.max_acceleration_scaling_factor = 0.3

        pc = PositionConstraint()
        pc.header.frame_id = self.base_frame
        pc.link_name = self.ee_link
        bv = BoundingVolume()
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.01]      # 半径 1cm（比原来放宽）
        bv.primitives.append(sphere)
        bv.primitive_poses.append(pose)
        pc.constraint_region = bv
        pc.weight = 1.0

        oc = OrientationConstraint()
        oc.header.frame_id = self.base_frame
        oc.link_name = self.ee_link
        oc.orientation = pose.orientation
        oc.absolute_x_axis_tolerance = 0.15   # 约 8.6°
        oc.absolute_y_axis_tolerance = 0.15
        oc.absolute_z_axis_tolerance = 0.15
        oc.weight = 1.0

        constraints = Constraints()
        constraints.position_constraints.append(pc)
        constraints.orientation_constraints.append(oc)
        req.goal_constraints.append(constraints)

        goal.request = req
        goal.planning_options.plan_only = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 3

        send_future = self._action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warn('  goal rejected')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result

        for _ in range(20):
            rclpy.spin_once(self, timeout_sec=0.1)

        if result.error_code.val != 1:
            self.get_logger().warn(f'  MoveIt error code: {result.error_code.val}')
        return result.error_code.val == 1

    # ---------- 在基准位姿附近随机扰动 ----------
    def random_perturb(self, base, max_angle_deg=25, max_trans=0.05):
        p = Pose()
        p.position.x = base.position.x + np.random.uniform(-max_trans, max_trans)
        p.position.y = base.position.y + np.random.uniform(-max_trans, max_trans)
        p.position.z = base.position.z + np.random.uniform(-max_trans, max_trans)

        q_base = [base.orientation.x, base.orientation.y,
                  base.orientation.z, base.orientation.w]
        rpy = tf_transformations.euler_from_quaternion(q_base)
        a = np.deg2rad(max_angle_deg)
        rpy = (rpy[0] + np.random.uniform(-a, a),
               rpy[1] + np.random.uniform(-a, a),
               rpy[2] + np.random.uniform(-a, a))
        q = tf_transformations.quaternion_from_euler(*rpy)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = q
        return p


def main():
    rclpy.init()
    node = AutoMover()

    # 拿一次初始位姿，作为扰动基准
    base = node.current_pose()
    if base is None:
        node.get_logger().error('无法从 TF 读取当前末端位姿，先确认机器人驱动和 TF 在跑。')
        rclpy.shutdown()
        return
    node.get_logger().info(
        f'Base EE pose: ({base.position.x:.3f}, '
        f'{base.position.y:.3f}, {base.position.z:.3f})')
    node.get_logger().info('开始自动移动。到 rqt 界面手动点 Take Sample 采样本。')
    node.get_logger().info('按 Ctrl+C 停止。')

    try:
        while rclpy.ok():
            target = node.random_perturb(base, max_angle_deg=25, max_trans=0.05)
            node.get_logger().info('planning...')
            if node.goto_pose(target):
                node.get_logger().info('  reached. 请手动点 Take Sample。')
                # 到位置后停 3 秒，留时间给你点按钮
                for _ in range(100):
                    rclpy.spin_once(node, timeout_sec=0.1)
            else:
                node.get_logger().warn('  motion planning failed, retry')
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()
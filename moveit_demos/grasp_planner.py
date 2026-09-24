#!/usr/bin/env python3
"""
抓取规划模块
功能：计算抓取位姿、生成接近轨迹
说明：
  - 抓取姿态："朝前"（末端 Z 轴指向基座 +X）
  - single_shot=True 时一次到位，无接近段
  - 支持手工 XYZ 偏移补偿（用于修正视觉/标定系统误差）
"""

import numpy as np
from typing import Tuple, List, Optional
from dataclasses import dataclass
import logging


@dataclass
class GraspPose:
    """抓取位姿数据类"""
    position: Tuple[float, float, float]
    orientation: Tuple[float, float, float, float]
    gripper_width: float
    approach_distance: float


@dataclass
class CameraToBaseTransform:
    """相机到机械臂基座的坐标变换"""
    translation: Tuple[float, float, float]
    rotation: np.ndarray

    @classmethod
    def from_xyz_rpy(cls, x: float, y: float, z: float,
                     roll: float, pitch: float, yaw: float):
        translation = (x, y, z)

        cr, sr = np.cos(roll), np.sin(roll)
        cp, sp = np.cos(pitch), np.sin(pitch)
        cy, sy = np.cos(yaw), np.sin(yaw)

        rotation = np.array([
            [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
            [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
            [-sp, cp*sr, cp*cr]
        ])

        return cls(translation=translation, rotation=rotation)


class GraspPlanner:
    """抓取规划器"""

    def __init__(self,
                 camera_to_base_transform: Optional[CameraToBaseTransform] = None,
                 default_gripper_width: float = 0.08,
                 approach_distance: float = 0.15,
                 grasp_depth_offset: float = 0.0):
        self.logger = logging.getLogger('GraspPlanner')

        if camera_to_base_transform is None:
            self.camera_to_base = CameraToBaseTransform.from_xyz_rpy(
                x=0.0, y=0.0, z=0.5,
                roll=0.0, pitch=np.pi, yaw=0.0
            )
            self.logger.warning("使用默认相机标定参数！请根据实际情况修改！")
        else:
            self.camera_to_base = camera_to_base_transform

        self.default_gripper_width = default_gripper_width
        self.approach_distance = approach_distance
        self.grasp_depth_offset = grasp_depth_offset

        # 工作空间限制（基座坐标系，米）
        self.workspace_limits = {
            'x': (0.15, 0.6),
            'y': (-0.4, 0.4),
            'z': (0.0, 0.5)
        }

        # 抓取姿态模式
        self.grasp_direction = 'forward'    # 'forward' | 'down'

        # 一次到位（无接近段）
        self.single_shot = True

        # 接近模式（single_shot=False 时生效）
        self.approach_mode = 'diagonal'     # 'diagonal' | 'aligned'

        # ==================================================================
        # 手工 XYZ 偏移补偿（关键！用于修正视觉/标定系统误差）
        # 单位：米，基座坐标系方向
        #   x > 0：目标位置向 +X 方向移动（远离基座）
        #   x < 0：目标位置向 -X 方向移动（靠近基座）
        # 调法：如果机器人"超出目标 15cm"，就设为 (-0.15, 0, 0)
        # ==================================================================
        self.target_offset = (0.0, 0.0, 0.0)

    # ==================================================================
    # 坐标变换
    # ==================================================================
    def camera_to_base_coordinates(self,
                                   x_cam: float,
                                   y_cam: float,
                                   z_cam: float) -> Tuple[float, float, float]:
        point_cam = np.array([x_cam, y_cam, z_cam])
        point_rotated = self.camera_to_base.rotation @ point_cam

        tx, ty, tz = self.camera_to_base.translation
        x_base = point_rotated[0] + tx
        y_base = point_rotated[1] + ty
        z_base = point_rotated[2] + tz

        self.logger.debug(f"坐标转换: 相机({x_cam:.3f}, {y_cam:.3f}, {z_cam:.3f}) "
                          f"-> 基座({x_base:.3f}, {y_base:.3f}, {z_base:.3f})")

        return x_base, y_base, z_base

    def is_in_workspace(self, x: float, y: float, z: float) -> bool:
        in_x = self.workspace_limits['x'][0] <= x <= self.workspace_limits['x'][1]
        in_y = self.workspace_limits['y'][0] <= y <= self.workspace_limits['y'][1]
        in_z = self.workspace_limits['z'][0] <= z <= self.workspace_limits['z'][1]
        return in_x and in_y and in_z

    # ==================================================================
    # 抓取位姿
    # ==================================================================
    def compute_grasp_pose(self,
                          x_cam: float,
                          y_cam: float,
                          z_cam: float,
                          gripper_width: Optional[float] = None,
                          grasp_angle: float = 0.0) -> Optional[GraspPose]:
        # 1. 相机坐标系 → 基座坐标系
        x_base, y_base, z_base = self.camera_to_base_coordinates(x_cam, y_cam, z_cam)

        # 2. 深度偏移（沿基座 Z 轴）
        z_grasp = z_base + self.grasp_depth_offset

        # 3. 手工 XYZ 补偿
        x_grasp = x_base + self.target_offset[0]
        y_grasp = y_base + self.target_offset[1]
        z_grasp = z_grasp + self.target_offset[2]

        self.logger.info(
            f"原始目标: ({x_base:.3f}, {y_base:.3f}, {z_base:.3f}) + "
            f"深度偏移 z={self.grasp_depth_offset:+.3f} + "
            f"手工偏移 {tuple(f'{v:+.3f}' for v in self.target_offset)} "
            f"→ 最终: ({x_grasp:.3f}, {y_grasp:.3f}, {z_grasp:.3f})"
        )

        if not self.is_in_workspace(x_grasp, y_grasp, z_grasp):
            self.logger.warning(
                f"目标位置 ({x_grasp:.3f}, {y_grasp:.3f}, {z_grasp:.3f}) 不在工作空间内"
            )
            return None

        orientation = self._compute_grasp_orientation(grasp_angle)

        if gripper_width is None:
            gripper_width = self.default_gripper_width

        grasp_pose = GraspPose(
            position=(x_grasp, y_grasp, z_grasp),
            orientation=orientation,
            gripper_width=gripper_width,
            approach_distance=self.approach_distance
        )

        self.logger.info(
            f"计算抓取位姿: 位置=({x_grasp:.3f}, {y_grasp:.3f}, {z_grasp:.3f}), "
            f"姿态=({orientation[0]:.3f}, {orientation[1]:.3f}, "
            f"{orientation[2]:.3f}, {orientation[3]:.3f}), "
            f"夹爪宽度={gripper_width:.3f}m"
        )

        return grasp_pose

    def _compute_grasp_orientation(self, grasp_angle: float = 0.0
                                   ) -> Tuple[float, float, float, float]:
        """forward: pitch=+π/2（朝前）；down: pitch=+π（朝下）"""
        roll = 0.0
        if self.grasp_direction == 'forward':
            pitch = np.pi / 2.0
        else:
            pitch = np.pi

        yaw = grasp_angle

        cy = np.cos(yaw * 0.5); sy = np.sin(yaw * 0.5)
        cp = np.cos(pitch * 0.5); sp = np.sin(pitch * 0.5)
        cr = np.cos(roll * 0.5);  sr = np.sin(roll * 0.5)

        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy

        return qx, qy, qz, qw

    # ==================================================================
    # 接近 / 撤退
    # ==================================================================
    def compute_approach_pose(self, grasp_pose: GraspPose) -> GraspPose:
        if self.single_shot:
            return GraspPose(
                position=grasp_pose.position,
                orientation=grasp_pose.orientation,
                gripper_width=grasp_pose.gripper_width,
                approach_distance=0.0
            )

        x, y, z = grasp_pose.position
        d = grasp_pose.approach_distance

        if self.grasp_direction == 'forward':
            if self.approach_mode == 'diagonal':
                offset_x = -d * np.cos(np.pi / 4)
                offset_z = +d * np.sin(np.pi / 4)
                approach_pos = (x + offset_x, y, z + offset_z)
            else:
                approach_pos = (x - d, y, z)
        else:
            approach_pos = (x, y, z + d)

        return GraspPose(
            position=approach_pos,
            orientation=grasp_pose.orientation,
            gripper_width=grasp_pose.gripper_width,
            approach_distance=grasp_pose.approach_distance
        )

    def compute_retreat_pose(self, grasp_pose: GraspPose,
                             retreat_distance: float = 0.15) -> GraspPose:
        x, y, z = grasp_pose.position

        if self.grasp_direction == 'forward':
            if self.approach_mode == 'diagonal' and not self.single_shot:
                offset_x = -retreat_distance * np.cos(np.pi / 4)
                offset_z = +retreat_distance * np.sin(np.pi / 4)
                retreat_pos = (x + offset_x, y, z + offset_z)
            else:
                retreat_pos = (x - retreat_distance, y, z)
        else:
            retreat_pos = (x, y, z + retreat_distance)

        return GraspPose(
            position=retreat_pos,
            orientation=grasp_pose.orientation,
            gripper_width=0.03,
            approach_distance=0.0
        )

    def generate_grasp_trajectory(self, grasp_pose: GraspPose) -> List[GraspPose]:
        approach_pose = self.compute_approach_pose(grasp_pose)
        retreat_pose = self.compute_retreat_pose(grasp_pose)
        trajectory = [approach_pose, grasp_pose, retreat_pose]

        self.logger.info(
            f"生成抓取轨迹: {len(trajectory)} 个路径点 "
            f"(single_shot={self.single_shot})"
        )
        return trajectory

    # ==================================================================
    # 参数
    # ==================================================================
    def set_workspace_limits(self, x_range: Tuple[float, float],
                             y_range: Tuple[float, float],
                             z_range: Tuple[float, float]):
        self.workspace_limits = {
            'x': x_range, 'y': y_range, 'z': z_range,
        }
        self.logger.info(f"更新工作空间限制: X={x_range}, Y={y_range}, Z={z_range}")

    def set_camera_transform(self, transform: CameraToBaseTransform):
        self.camera_to_base = transform
        self.logger.info("更新相机标定参数")

    def set_target_offset(self, dx: float, dy: float, dz: float):
        """设置手工 XYZ 偏移（米）"""
        self.target_offset = (dx, dy, dz)
        self.logger.info(f"更新目标偏移: ({dx:+.3f}, {dy:+.3f}, {dz:+.3f})")

    def estimate_object_size(self, depth_image,
                             bbox: Tuple[int, int, int, int],
                             depth_scale: float) -> Tuple[float, float]:
        x, y, w, h = bbox
        depth_roi = depth_image[y:y+h, x:x+w]
        valid_depths = depth_roi[depth_roi > 0]

        if len(valid_depths) == 0:
            return 0.08, 0.08

        avg_depth = np.mean(valid_depths) * depth_scale
        focal_length = 320.0

        width_meters = (w * avg_depth) / focal_length
        height_meters = (h * avg_depth) / focal_length

        return width_meters, height_meters


def test_planner():
    import logging
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )
    planner = GraspPlanner()
    planner.grasp_direction = 'forward'
    planner.single_shot = True
    planner.set_target_offset(-0.15, 0.0, 0.0)   # 演示 -15cm 补偿

    x_cam, y_cam, z_cam = 0.0, 0.0, 0.5
    grasp_pose = planner.compute_grasp_pose(x_cam, y_cam, z_cam)
    if grasp_pose:
        print(f"抓取位置: {grasp_pose.position}")
        print(f"抓取姿态: {grasp_pose.orientation}")


if __name__ == '__main__':
    test_planner()
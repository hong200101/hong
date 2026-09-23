#!/usr/bin/env python3
"""
抓取规划模块
功能：计算抓取位姿、生成接近轨迹
"""

import numpy as np
from typing import Tuple, List, Optional
from dataclasses import dataclass
import logging


@dataclass
class GraspPose:
    """抓取位姿数据类"""
    position: Tuple[float, float, float]  # (x, y, z) 位置（米）
    orientation: Tuple[float, float, float, float]  # (x, y, z, w) 四元数
    gripper_width: float  # 夹爪开口宽度（米）
    approach_distance: float  # 接近距离（米）


@dataclass
class CameraToBaseTransform:
    """相机到机械臂基座的坐标变换"""
    translation: Tuple[float, float, float]  # (x, y, z) 平移（米）
    rotation: np.ndarray  # 3x3 旋转矩阵
    
    @classmethod
    def from_xyz_rpy(cls, x: float, y: float, z: float, 
                     roll: float, pitch: float, yaw: float):
        """
        从位置和欧拉角创建变换
        
        Args:
            x, y, z: 平移（米）
            roll, pitch, yaw: 旋转（弧度）
        """
        translation = (x, y, z)
        
        # 计算旋转矩阵
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
        """
        初始化抓取规划器
        
        Args:
            camera_to_base_transform: 相机到基座的坐标变换
            default_gripper_width: 默认夹爪开口宽度（米）
            approach_distance: 接近距离（米）
            grasp_depth_offset: 抓取深度偏移（米，正值向下）
        """
        self.logger = logging.getLogger('GraspPlanner')
        
        # 设置相机到基座的变换（默认值，需要根据实际标定）
        if camera_to_base_transform is None:
            # 默认配置：相机在机械臂上方，向下看
            # 需要根据实际安装位置进行标定！
            self.camera_to_base = CameraToBaseTransform.from_xyz_rpy(
                x=0.0, y=0.0, z=0.5,  # 相机位置（示例值）
                roll=0.0, pitch=np.pi, yaw=0.0  # 相机朝向（向下）
            )
            self.logger.warning("使用默认相机标定参数！请根据实际情况修改！")
        else:
            self.camera_to_base = camera_to_base_transform
        
        self.default_gripper_width = default_gripper_width
        self.approach_distance = approach_distance
        self.grasp_depth_offset = grasp_depth_offset
        
        # 工作空间限制（基座坐标系，米）
        self.workspace_limits = {
            'x': (0.15, 0.6),   # X 范围
            'y': (-0.4, 0.4),   # Y 范围
            'z': (0.0, 0.5)     # Z 范围
        }
        
    def camera_to_base_coordinates(self, 
                                   x_cam: float, 
                                   y_cam: float, 
                                   z_cam: float) -> Tuple[float, float, float]:
        """
        将相机坐标系下的点转换到机械臂基座坐标系
        
        Args:
            x_cam, y_cam, z_cam: 相机坐标系下的位置（米）
            
        Returns:
            Tuple[x_base, y_base, z_base]: 基座坐标系下的位置（米）
        """
        # 相机坐标系下的点
        point_cam = np.array([x_cam, y_cam, z_cam])
        
        # 应用旋转
        point_rotated = self.camera_to_base.rotation @ point_cam
        
        # 应用平移
        tx, ty, tz = self.camera_to_base.translation
        x_base = point_rotated[0] + tx
        y_base = point_rotated[1] + ty
        z_base = point_rotated[2] + tz
        
        self.logger.debug(f"坐标转换: 相机({x_cam:.3f}, {y_cam:.3f}, {z_cam:.3f}) "
                         f"-> 基座({x_base:.3f}, {y_base:.3f}, {z_base:.3f})")
        
        return x_base, y_base, z_base
    
    def is_in_workspace(self, x: float, y: float, z: float) -> bool:
        """
        检查位置是否在工作空间内
        
        Args:
            x, y, z: 基座坐标系下的位置（米）
            
        Returns:
            bool: 在工作空间内返回 True
        """
        in_x = self.workspace_limits['x'][0] <= x <= self.workspace_limits['x'][1]
        in_y = self.workspace_limits['y'][0] <= y <= self.workspace_limits['y'][1]
        in_z = self.workspace_limits['z'][0] <= z <= self.workspace_limits['z'][1]
        
        return in_x and in_y and in_z
    
    def compute_grasp_pose(self, 
                          x_cam: float, 
                          y_cam: float, 
                          z_cam: float,
                          gripper_width: Optional[float] = None,
                          grasp_angle: float = 0.0) -> Optional[GraspPose]:
        """
        计算抓取位姿
        
        Args:
            x_cam, y_cam, z_cam: 物体在相机坐标系下的位置（米）
            gripper_width: 夹爪开口宽度（米），None 使用默认值
            grasp_angle: 抓取旋转角度（弧度，绕 Z 轴）
            
        Returns:
            Optional[GraspPose]: 抓取位姿，如果不可达返回 None
        """
        # 转换到基座坐标系
        x_base, y_base, z_base = self.camera_to_base_coordinates(x_cam, y_cam, z_cam)
        
        # 应用抓取深度偏移
        z_grasp = z_base + self.grasp_depth_offset
        
        # 检查是否在工作空间内
        if not self.is_in_workspace(x_base, y_base, z_grasp):
            self.logger.warning(f"目标位置 ({x_base:.3f}, {y_base:.3f}, {z_grasp:.3f}) 不在工作空间内")
            return None
        
        # 计算抓取姿态（末端执行器垂直向下，可加旋转角）
        # 使用四元数表示姿态
        orientation = self._compute_grasp_orientation(grasp_angle)
        
        # 设置夹爪宽度
        if gripper_width is None:
            gripper_width = self.default_gripper_width
        
        grasp_pose = GraspPose(
            position=(x_base, y_base, z_grasp),
            orientation=orientation,
            gripper_width=gripper_width,
            approach_distance=self.approach_distance
        )
        
        self.logger.info(f"计算抓取位姿: 位置=({x_base:.3f}, {y_base:.3f}, {z_grasp:.3f}), "
                        f"夹爪宽度={gripper_width:.3f}m")
        
        return grasp_pose
    
    def _compute_grasp_orientation(self, grasp_angle: float = 0.0) -> Tuple[float, float, float, float]:
        """
        计算抓取姿态四元数（末端执行器垂直向下）
        
        Args:
            grasp_angle: 绕 Z 轴旋转角度（弧度）
            
        Returns:
            Tuple[qx, qy, qz, qw]: 四元数
        """
        # 垂直向下：roll=0, pitch=π, yaw=grasp_angle
        roll = 0.0
        pitch = np.pi
        yaw = grasp_angle
        
        # 欧拉角转四元数
        cy = np.cos(yaw * 0.5)
        sy = np.sin(yaw * 0.5)
        cp = np.cos(pitch * 0.5)
        sp = np.sin(pitch * 0.5)
        cr = np.cos(roll * 0.5)
        sr = np.sin(roll * 0.5)
        
        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        
        return qx, qy, qz, qw
    
    def compute_approach_pose(self, grasp_pose: GraspPose) -> GraspPose:
        """
        计算接近位姿（抓取位姿上方）
        
        Args:
            grasp_pose: 抓取位姿
            
        Returns:
            GraspPose: 接近位姿
        """
        x, y, z = grasp_pose.position
        
        # 接近位姿在抓取位姿上方
        approach_pose = GraspPose(
            position=(x, y, z + grasp_pose.approach_distance),
            orientation=grasp_pose.orientation,
            gripper_width=grasp_pose.gripper_width,
            approach_distance=grasp_pose.approach_distance
        )
        
        self.logger.debug(f"计算接近位姿: ({x:.3f}, {y:.3f}, {z + grasp_pose.approach_distance:.3f})")
        
        return approach_pose
    
    def compute_retreat_pose(self, grasp_pose: GraspPose, retreat_distance: float = 0.15) -> GraspPose:
        """
        计算撤退位姿（抓取后上升）
        
        Args:
            grasp_pose: 抓取位姿
            retreat_distance: 撤退距离（米）
            
        Returns:
            GraspPose: 撤退位姿
        """
        x, y, z = grasp_pose.position
        
        retreat_pose = GraspPose(
            position=(x, y, z + retreat_distance),
            orientation=grasp_pose.orientation,
            gripper_width=0.03,  # 抓取后夹爪闭合
            approach_distance=0.0
        )
        
        self.logger.debug(f"计算撤退位姿: ({x:.3f}, {y:.3f}, {z + retreat_distance:.3f})")
        
        return retreat_pose
    
    def generate_grasp_trajectory(self, 
                                 grasp_pose: GraspPose) -> List[GraspPose]:
        """
        生成完整的抓取轨迹
        
        Args:
            grasp_pose: 目标抓取位姿
            
        Returns:
            List[GraspPose]: 抓取轨迹点列表 [接近, 抓取, 撤退]
        """
        trajectory = []
        
        # 1. 接近位姿
        approach_pose = self.compute_approach_pose(grasp_pose)
        trajectory.append(approach_pose)
        
        # 2. 抓取位姿
        trajectory.append(grasp_pose)
        
        # 3. 撤退位姿
        retreat_pose = self.compute_retreat_pose(grasp_pose)
        trajectory.append(retreat_pose)
        
        self.logger.info(f"生成抓取轨迹: {len(trajectory)} 个路径点")
        
        return trajectory
    
    def set_workspace_limits(self, x_range: Tuple[float, float], 
                           y_range: Tuple[float, float],
                           z_range: Tuple[float, float]):
        """
        设置工作空间限制
        
        Args:
            x_range, y_range, z_range: 各轴的范围 (min, max)
        """
        self.workspace_limits = {
            'x': x_range,
            'y': y_range,
            'z': z_range
        }
        self.logger.info(f"更新工作空间限制: X={x_range}, Y={y_range}, Z={z_range}")
    
    def set_camera_transform(self, transform: CameraToBaseTransform):
        """
        设置相机到基座的坐标变换
        
        Args:
            transform: 坐标变换对象
        """
        self.camera_to_base = transform
        self.logger.info("更新相机标定参数")
    
    def estimate_object_size(self, 
                           depth_image, 
                           bbox: Tuple[int, int, int, int],
                           depth_scale: float) -> Tuple[float, float]:
        """
        估计物体尺寸（用于自适应夹爪宽度）
        
        Args:
            depth_image: 深度图像
            bbox: 边界框 (x, y, w, h)
            depth_scale: 深度单位
            
        Returns:
            Tuple[width, height]: 物体宽度和高度（米）
        """
        x, y, w, h = bbox
        
        # 简化估计：基于像素尺寸和平均深度
        depth_roi = depth_image[y:y+h, x:x+w]
        valid_depths = depth_roi[depth_roi > 0]
        
        if len(valid_depths) == 0:
            return 0.08, 0.08  # 默认值
        
        avg_depth = np.mean(valid_depths) * depth_scale
        
        # 使用针孔相机模型估计实际尺寸
        # 假设相机焦距约为 width/2（简化）
        focal_length = 320.0  # 根据实际相机参数调整
        
        width_meters = (w * avg_depth) / focal_length
        height_meters = (h * avg_depth) / focal_length
        
        self.logger.debug(f"估计物体尺寸: 宽={width_meters:.3f}m, 高={height_meters:.3f}m")
        
        return width_meters, height_meters


def test_planner():
    """测试抓取规划器"""
    import logging
    
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )
    
    logger = logging.getLogger('PlannerTest')
    
    # 创建规划器（使用默认相机标定）
    planner = GraspPlanner()
    
    # 模拟相机检测到的物体位置（相机坐标系）
    x_cam, y_cam, z_cam = 0.0, 0.0, 0.5  # 物体在相机前方 0.5 米
    
    logger.info(f"模拟检测到物体: 相机坐标=({x_cam}, {y_cam}, {z_cam})")
    
    # 计算抓取位姿
    grasp_pose = planner.compute_grasp_pose(x_cam, y_cam, z_cam)
    
    if grasp_pose is None:
        logger.error("无法计算抓取位姿")
        return
    
    logger.info(f"抓取位姿: 位置={grasp_pose.position}, "
               f"姿态={grasp_pose.orientation}, "
               f"夹爪={grasp_pose.gripper_width}m")
    
    # 生成完整轨迹
    trajectory = planner.generate_grasp_trajectory(grasp_pose)
    
    logger.info(f"\n抓取轨迹 ({len(trajectory)} 个点):")
    for i, pose in enumerate(trajectory):
        logger.info(f"  点 {i+1}: 位置={pose.position}, 夹爪={pose.gripper_width}m")


if __name__ == '__main__':
    test_planner()

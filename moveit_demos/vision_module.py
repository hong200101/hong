#!/usr/bin/env python3
"""
RealSense D435 视觉处理模块
功能：检测蓝色物体并获取其 3D 位置
"""

import numpy as np
import cv2
import pyrealsense2 as rs
from typing import Optional, Tuple, List
import logging


class RealSenseVision:
    """RealSense D435 相机视觉处理类"""
    
    def __init__(self, 
                 width: int = 640, 
                 height: int = 480, 
                 fps: int = 30,
                 enable_filter: bool = True):
        """
        初始化 RealSense 相机
        
        Args:
            width: 图像宽度
            height: 图像高度
            fps: 帧率
            enable_filter: 是否启用深度滤波器
        """
        self.width = width
        self.height = height
        self.fps = fps
        self.enable_filter = enable_filter
        
        # 初始化 RealSense 管道
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        
        # 配置流
        self.config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
        self.config.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
        
        # 对齐对象（将深度图对齐到彩色图）
        self.align = rs.align(rs.stream.color)
        
        # 深度滤波器
        if enable_filter:
            self.spatial_filter = rs.spatial_filter()
            self.temporal_filter = rs.temporal_filter()
            self.hole_filling_filter = rs.hole_filling_filter()
            
        self.logger = logging.getLogger('RealSenseVision')
        self.is_streaming = False
        
        # 蓝色检测 HSV 范围（可调整）
        self.blue_lower = np.array([90, 50, 50])    # HSV 下界
        self.blue_upper = np.array([130, 255, 255])  # HSV 上界
        
        # 最小检测面积（像素）
        self.min_area = 500
        
    def start(self) -> bool:
        """
        启动相机流
        
        Returns:
            bool: 成功返回 True
        """
        try:
            # 启动管道
            profile = self.pipeline.start(self.config)
            
            # 获取深度传感器的深度单位
            depth_sensor = profile.get_device().first_depth_sensor()
            self.depth_scale = depth_sensor.get_depth_scale()
            
            self.logger.info(f"相机启动成功! 深度单位: {self.depth_scale}")
            self.is_streaming = True
            
            # 预热相机（跳过前几帧）
            for _ in range(30):
                self.pipeline.wait_for_frames()
            
            return True
            
        except Exception as e:
            self.logger.error(f"相机启动失败: {e}")
            return False
    
    def stop(self):
        """停止相机流"""
        if self.is_streaming:
            self.pipeline.stop()
            self.is_streaming = False
            self.logger.info("相机已停止")
    
    def get_frames(self) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """
        获取对齐的彩色图和深度图
        
        Returns:
            Tuple[color_image, depth_image]: 彩色图和深度图，失败返回 (None, None)
        """
        if not self.is_streaming:
            self.logger.warning("相机未启动")
            return None, None
        
        try:
            # 等待新的一帧
            frames = self.pipeline.wait_for_frames()
            
            # 对齐深度图到彩色图
            aligned_frames = self.align.process(frames)
            
            # 获取对齐的帧
            color_frame = aligned_frames.get_color_frame()
            depth_frame = aligned_frames.get_depth_frame()
            
            if not color_frame or not depth_frame:
                return None, None
            
            # 应用深度滤波器
            if self.enable_filter:
                depth_frame = self.spatial_filter.process(depth_frame)
                depth_frame = self.temporal_filter.process(depth_frame)
                depth_frame = self.hole_filling_filter.process(depth_frame)
            
            # 转换为 numpy 数组
            color_image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data())
            
            return color_image, depth_image
            
        except Exception as e:
            self.logger.error(f"获取帧失败: {e}")
            return None, None
    
    def detect_blue_object(self, 
                          color_image: np.ndarray,
                          visualize: bool = False) -> Optional[Tuple[int, int, int, int]]:
        """
        检测蓝色物体并返回边界框
        
        Args:
            color_image: 输入彩色图像
            visualize: 是否可视化检测结果
            
        Returns:
            Optional[Tuple[x, y, w, h]]: 检测到的物体边界框 (x, y, width, height)，未检测到返回 None
        """
        # 转换到 HSV 色彩空间
        hsv = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)
        
        # 创建蓝色掩码
        mask = cv2.inRange(hsv, self.blue_lower, self.blue_upper)
        
        # 形态学操作去除噪声
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        
        # 查找轮廓
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if not contours:
            return None
        
        # 找到最大的轮廓
        largest_contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest_contour)
        
        # 检查面积是否满足最小要求
        if area < self.min_area:
            self.logger.debug(f"检测到的物体面积太小: {area} < {self.min_area}")
            return None
        
        # 获取边界框
        x, y, w, h = cv2.boundingRect(largest_contour)
        
        # 可视化
        if visualize:
            result = color_image.copy()
            cv2.rectangle(result, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(result, f"Area: {int(area)}", (x, y - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
            cv2.imshow("Blue Object Detection", result)
            cv2.imshow("Mask", mask)
            cv2.waitKey(1)
        
        self.logger.info(f"检测到蓝色物体: 位置=({x}, {y}), 大小=({w}, {h}), 面积={area:.0f}")
        return x, y, w, h
    
    def get_3d_position(self, 
                       depth_image: np.ndarray,
                       x: int, y: int, w: int, h: int,
                       use_median: bool = True) -> Optional[Tuple[float, float, float]]:
        """
        获取物体的 3D 位置（相机坐标系）
        
        Args:
            depth_image: 深度图像
            x, y, w, h: 物体边界框
            use_median: 是否使用中值深度（更鲁棒）
            
        Returns:
            Optional[Tuple[x, y, z]]: 3D 位置（米），失败返回 None
        """
        # 计算物体中心
        cx = x + w // 2
        cy = y + h // 2
        
        # 提取物体区域的深度值
        depth_roi = depth_image[y:y+h, x:x+w]
        
        # 过滤无效深度值（0 值）
        valid_depths = depth_roi[depth_roi > 0]
        
        if len(valid_depths) == 0:
            self.logger.warning("物体区域没有有效深度值")
            return None
        
        # 使用中值深度或平均深度
        if use_median:
            depth_value = np.median(valid_depths)
        else:
            depth_value = np.mean(valid_depths)
        
        # 转换为米
        depth_meters = depth_value * self.depth_scale
        
        # 获取相机内参
        profile = self.pipeline.get_active_profile()
        depth_intrinsics = profile.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()
        
        # 像素坐标转 3D 坐标
        point_3d = rs.rs2_deproject_pixel_to_point(depth_intrinsics, [cx, cy], depth_meters)
        
        x_cam, y_cam, z_cam = point_3d
        
        self.logger.info(f"物体 3D 位置 (相机坐标系): x={x_cam:.3f}m, y={y_cam:.3f}m, z={z_cam:.3f}m")
        
        return x_cam, y_cam, z_cam
    
    def detect_and_locate(self, visualize: bool = False) -> Optional[Tuple[float, float, float]]:
        """
        检测蓝色物体并返回其 3D 位置（一站式接口）
        
        Args:
            visualize: 是否可视化
            
        Returns:
            Optional[Tuple[x, y, z]]: 物体 3D 位置（相机坐标系，米），未检测到返回 None
        """
        # 获取图像
        color_image, depth_image = self.get_frames()
        
        if color_image is None or depth_image is None:
            return None
        
        # 检测蓝色物体
        bbox = self.detect_blue_object(color_image, visualize)
        
        if bbox is None:
            return None
        
        x, y, w, h = bbox
        
        # 获取 3D 位置
        position_3d = self.get_3d_position(depth_image, x, y, w, h)
        
        return position_3d
    
    def set_blue_range(self, lower: Tuple[int, int, int], upper: Tuple[int, int, int]):
        """
        设置蓝色检测的 HSV 范围
        
        Args:
            lower: HSV 下界 (H, S, V)
            upper: HSV 上界 (H, S, V)
        """
        self.blue_lower = np.array(lower)
        self.blue_upper = np.array(upper)
        self.logger.info(f"更新蓝色范围: {lower} - {upper}")
    
    def set_min_area(self, area: int):
        """
        设置最小检测面积
        
        Args:
            area: 最小面积（像素）
        """
        self.min_area = area
        self.logger.info(f"更新最小检测面积: {area}")
    
    def capture_snapshot(self, filename: str = "snapshot.png"):
        """
        捕获当前帧并保存
        
        Args:
            filename: 保存文件名
        """
        color_image, _ = self.get_frames()
        if color_image is not None:
            cv2.imwrite(filename, color_image)
            self.logger.info(f"快照已保存: {filename}")
    
    def __del__(self):
        """析构函数"""
        self.stop()
        cv2.destroyAllWindows()


def test_vision():
    """测试视觉模块"""
    import logging
    
    # 配置日志
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )
    
    logger = logging.getLogger('VisionTest')
    
    # 创建视觉对象
    vision = RealSenseVision(width=640, height=480, fps=30)
    
    # 启动相机
    if not vision.start():
        logger.error("无法启动相机")
        return
    
    logger.info("相机已启动，开始检测蓝色物体...")
    logger.info("按 'q' 退出, 's' 保存快照, 'c' 校准颜色范围")
    
    try:
        calibrating = False
        
        while True:
            # 检测并定位物体
            position = vision.detect_and_locate(visualize=True)
            
            if position is not None:
                x, y, z = position
                logger.info(f"检测到物体: ({x:.3f}, {y:.3f}, {z:.3f}) 米")
            
            # 键盘控制
            key = cv2.waitKey(1) & 0xFF
            
            if key == ord('q'):
                logger.info("退出程序")
                break
            elif key == ord('s'):
                vision.capture_snapshot(f"snapshot_{int(cv2.getTickCount())}.png")
            elif key == ord('c'):
                logger.info("进入颜色校准模式...")
                calibrating = not calibrating
                if calibrating:
                    # 创建颜色校准窗口
                    cv2.namedWindow('Color Calibration')
                    cv2.createTrackbar('H_min', 'Color Calibration', 90, 179, lambda x: None)
                    cv2.createTrackbar('H_max', 'Color Calibration', 130, 179, lambda x: None)
                    cv2.createTrackbar('S_min', 'Color Calibration', 50, 255, lambda x: None)
                    cv2.createTrackbar('S_max', 'Color Calibration', 255, 255, lambda x: None)
                    cv2.createTrackbar('V_min', 'Color Calibration', 50, 255, lambda x: None)
                    cv2.createTrackbar('V_max', 'Color Calibration', 255, 255, lambda x: None)
                else:
                    cv2.destroyWindow('Color Calibration')
            
            # 应用校准值
            if calibrating:
                h_min = cv2.getTrackbarPos('H_min', 'Color Calibration')
                h_max = cv2.getTrackbarPos('H_max', 'Color Calibration')
                s_min = cv2.getTrackbarPos('S_min', 'Color Calibration')
                s_max = cv2.getTrackbarPos('S_max', 'Color Calibration')
                v_min = cv2.getTrackbarPos('V_min', 'Color Calibration')
                v_max = cv2.getTrackbarPos('V_max', 'Color Calibration')
                
                vision.set_blue_range((h_min, s_min, v_min), (h_max, s_max, v_max))
    
    except KeyboardInterrupt:
        logger.info("用户中断")
    
    finally:
        vision.stop()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    test_vision()

#!/usr/bin/env python3
"""
相机和蓝色检测测试工具
用于调试相机连接和颜色检测参数
"""

import cv2
import numpy as np
import pyrealsense2 as rs
import sys


class CameraDetectionTester:
    def __init__(self):
        self.pipeline = None
        self.align = None
        
        # 蓝色检测参数 (可调整)
        self.blue_lower = np.array([90, 80, 80])
        self.blue_upper = np.array([130, 255, 255])
        
        print("="*60)
        print("相机和蓝色检测测试工具")
        print("="*60)
    
    def initialize_camera(self):
        """初始化相机"""
        print("\n初始化 RealSense D435...")
        
        try:
            self.pipeline = rs.pipeline()
            config = rs.config()
            
            config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
            config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
            
            profile = self.pipeline.start(config)
            
            align_to = rs.stream.color
            self.align = rs.align(align_to)
            
            depth_sensor = profile.get_device().first_depth_sensor()
            self.depth_scale = depth_sensor.get_depth_scale()
            
            print(f"✓ 相机初始化成功! 深度比例: {self.depth_scale}")
            
            # 预热
            for _ in range(30):
                self.pipeline.wait_for_frames()
            
            return True
            
        except Exception as e:
            print(f"✗ 相机初始化失败: {e}")
            return False
    
    def get_frames(self):
        """获取图像帧"""
        try:
            frames = self.pipeline.wait_for_frames()
            aligned_frames = self.align.process(frames)
            
            color_frame = aligned_frames.get_color_frame()
            depth_frame = aligned_frames.get_depth_frame()
            
            if not color_frame or not depth_frame:
                return None, None, None
            
            color_image = np.asanyarray(color_frame.data)
            depth_image = np.asanyarray(depth_frame.data)
            
            return color_image, depth_image, depth_frame
            
        except Exception as e:
            print(f"获取帧失败: {e}")
            return None, None, None
    
    def detect_blue(self, color_image, depth_frame):
        """检测蓝色物体"""
        hsv = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.blue_lower, self.blue_upper)
        
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        result_image = color_image.copy()
        detection_info = []
        
        for contour in contours:
            area = cv2.contourArea(contour)
            
            if area < 500:
                continue
            
            # 绘制轮廓
            cv2.drawContours(result_image, [contour], -1, (0, 255, 0), 2)
            
            # 计算中心
            M = cv2.moments(contour)
            if M['m00'] != 0:
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                
                # 获取深度
                depth = depth_frame.get_distance(cx, cy)
                
                # 绘制中心点
                cv2.circle(result_image, (cx, cy), 5, (255, 0, 0), -1)
                
                # 获取3D坐标
                if depth > 0:
                    intrinsics = depth_frame.profile.as_video_stream_profile().intrinsics
                    point_3d = rs.rs2_deproject_pixel_to_point(intrinsics, [cx, cy], depth)
                    
                    info_text = f"({cx}, {cy}): depth={depth:.3f}m"
                    cv2.putText(result_image, info_text, (cx + 10, cy), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)
                    
                    detection_info.append({
                        'pixel': (cx, cy),
                        'depth': depth,
                        'area': area,
                        '3d_camera': point_3d
                    })
        
        return result_image, mask, detection_info
    
    def adjust_hsv_callback(self, x):
        """HSV 调整滑动条回调"""
        pass
    
    def run_test(self):
        """运行测试"""
        if not self.initialize_camera():
            return
        
        print("\n="*60)
        print("控制说明:")
        print("  q - 退出")
        print("  s - 保存当前图像")
        print("  h - 显示/隐藏 HSV 调整窗口")
        print("="*60)
        
        # 创建窗口
        cv2.namedWindow('Color Image')
        cv2.namedWindow('Blue Detection')
        cv2.namedWindow('Mask')
        
        # 创建 HSV 调整窗口
        cv2.namedWindow('HSV Adjustment')
        cv2.createTrackbar('H Lower', 'HSV Adjustment', self.blue_lower[0], 179, self.adjust_hsv_callback)
        cv2.createTrackbar('S Lower', 'HSV Adjustment', self.blue_lower[1], 255, self.adjust_hsv_callback)
        cv2.createTrackbar('V Lower', 'HSV Adjustment', self.blue_lower[2], 255, self.adjust_hsv_callback)
        cv2.createTrackbar('H Upper', 'HSV Adjustment', self.blue_upper[0], 179, self.adjust_hsv_callback)
        cv2.createTrackbar('S Upper', 'HSV Adjustment', self.blue_upper[1], 255, self.adjust_hsv_callback)
        cv2.createTrackbar('V Upper', 'HSV Adjustment', self.blue_upper[2], 255, self.adjust_hsv_callback)
        
        frame_count = 0
        show_hsv_window = True
        
        while True:
            # 获取图像
            color_image, depth_image, depth_frame = self.get_frames()
            
            if color_image is None:
                continue
            
            # 从滑动条读取 HSV 参数
            h_lower = cv2.getTrackbarPos('H Lower', 'HSV Adjustment')
            s_lower = cv2.getTrackbarPos('S Lower', 'HSV Adjustment')
            v_lower = cv2.getTrackbarPos('V Lower', 'HSV Adjustment')
            h_upper = cv2.getTrackbarPos('H Upper', 'HSV Adjustment')
            s_upper = cv2.getTrackbarPos('S Upper', 'HSV Adjustment')
            v_upper = cv2.getTrackbarPos('V Upper', 'HSV Adjustment')
            
            self.blue_lower = np.array([h_lower, s_lower, v_lower])
            self.blue_upper = np.array([h_upper, s_upper, v_upper])
            
            # 检测蓝色
            result_image, mask, detection_info = self.detect_blue(color_image, depth_frame)
            
            # 显示检测数量
            info_text = f"Detections: {len(detection_info)} | Frame: {frame_count}"
            cv2.putText(result_image, info_text, (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            
            # 在控制台打印检测信息
            if len(detection_info) > 0 and frame_count % 30 == 0:
                print(f"\n[Frame {frame_count}] 检测到 {len(detection_info)} 个蓝色物体:")
                for i, info in enumerate(detection_info):
                    print(f"  {i+1}. 像素: {info['pixel']}, "
                          f"深度: {info['depth']:.3f}m, "
                          f"面积: {info['area']:.0f}px, "
                          f"3D: ({info['3d_camera'][0]:.3f}, {info['3d_camera'][1]:.3f}, {info['3d_camera'][2]:.3f})")
            
            # 显示图像
            cv2.imshow('Color Image', color_image)
            cv2.imshow('Blue Detection', result_image)
            cv2.imshow('Mask', mask)
            
            # 键盘控制
            key = cv2.waitKey(1) & 0xFF
            
            if key == ord('q'):
                print("\n退出测试...")
                break
            elif key == ord('s'):
                # 保存图像
                cv2.imwrite(f'color_{frame_count}.png', color_image)
                cv2.imwrite(f'detection_{frame_count}.png', result_image)
                cv2.imwrite(f'mask_{frame_count}.png', mask)
                print(f"\n✓ 已保存图像 (frame {frame_count})")
            elif key == ord('h'):
                # 切换 HSV 窗口显示
                show_hsv_window = not show_hsv_window
                if show_hsv_window:
                    cv2.namedWindow('HSV Adjustment')
                else:
                    cv2.destroyWindow('HSV Adjustment')
            
            frame_count += 1
        
        # 打印最终参数
        print("\n最终 HSV 参数:")
        print(f"  blue_lower = np.array([{h_lower}, {s_lower}, {v_lower}])")
        print(f"  blue_upper = np.array([{h_upper}, {s_upper}, {v_upper}])")
        
        self.cleanup()
    
    def cleanup(self):
        """清理资源"""
        if self.pipeline:
            self.pipeline.stop()
        cv2.destroyAllWindows()
        print("\n✓ 测试完成")


def main():
    try:
        tester = CameraDetectionTester()
        tester.run_test()
    except KeyboardInterrupt:
        print("\n用户中断")
    except Exception as e:
        print(f"\n✗ 错误: {e}")
        import traceback
        traceback.print_exc()


if __name__ == '__main__':
    main()

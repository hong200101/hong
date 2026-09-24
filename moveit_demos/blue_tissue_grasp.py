#!/usr/bin/env python3
"""
蓝色纸巾包视觉抓取主程序（纯 MoveIt 版）
功能：整合视觉识别、抓取规划和 MoveIt 位姿规划，实现完整的抓取流程
说明：不使用 MIT 柔顺控制，全程 MoveIt 位姿规划；不设初始位置，流程开始/结束均回零位
"""

import sys
import rclpy
from rclpy.node import Node
import time
import numpy as np
import logging
from typing import Optional, Tuple
import yaml
import os

# 导入自定义模块
from vision_module import RealSenseVision
from grasp_planner import GraspPlanner, CameraToBaseTransform, GraspPose
from hybrid_controller import HybridController


class BlueTissueGraspSystem(Node):
    """蓝色纸巾包抓取系统（纯 MoveIt）"""

    def __init__(self, config_file: Optional[str] = None):
        super().__init__('blue_tissue_grasp_system')

        self.logger = logging.getLogger('BlueTissueGrasp')
        self.logger.info('=' * 70)
        self.logger.info('初始化蓝色纸巾包视觉抓取系统（纯 MoveIt）')
        self.logger.info('=' * 70)

        # 加载配置
        self.config = self._load_config(config_file)

        # 初始化视觉模块
        self.logger.info('初始化视觉模块...')
        self.vision = RealSenseVision(
            width=self.config['vision']['width'],
            height=self.config['vision']['height'],
            fps=self.config['vision']['fps'],
            enable_filter=self.config['vision']['enable_filter']
        )

        blue_lower = tuple(self.config['vision']['blue_hsv_lower'])
        blue_upper = tuple(self.config['vision']['blue_hsv_upper'])
        self.vision.set_blue_range(blue_lower, blue_upper)
        self.vision.set_min_area(self.config['vision']['min_area'])

        # 初始化抓取规划器
        self.logger.info('初始化抓取规划器...')
        camera_transform = self._create_camera_transform()
        self.planner = GraspPlanner(
            camera_to_base_transform=camera_transform,
            default_gripper_width=self.config['grasp']['default_gripper_width'],
            approach_distance=self.config['grasp']['approach_distance'],
            grasp_depth_offset=self.config['grasp']['grasp_depth_offset']
        )

        ws = self.config['grasp']['workspace_limits']
        self.planner.set_workspace_limits(
            tuple(ws['x']), tuple(ws['y']), tuple(ws['z'])
        )

        # 初始化 MoveIt 控制器
        self.logger.info('初始化 MoveIt 控制器...')
        self.controller = HybridController(node_name='tissue_grasp_controller')

        self.is_vision_started = False
        self.current_target_position = None

        self.logger.info('=' * 70)
        self.logger.info('✓ 系统初始化完成!')
        self.logger.info('=' * 70)

    # ==================================================================
    # 配置加载
    # ==================================================================
    def _load_config(self, config_file: Optional[str] = None) -> dict:
        default_config = {
            'vision': {
                'width': 640,
                'height': 480,
                'fps': 30,
                'enable_filter': True,
                'blue_hsv_lower': [90, 50, 50],
                'blue_hsv_upper': [130, 255, 255],
                'min_area': 500,
            },
            'camera_calibration': {
                'position': [0.4, 0.0, 0.5],
                'orientation': [0.0, 3.14159, 0.0],
            },
            'grasp': {
                'default_gripper_width': 0.08,
                'approach_distance': 0.15,
                'grasp_depth_offset': -0.05,
                'workspace_limits': {
                    'x': [0.15, 0.6],
                    'y': [-0.4, 0.4],
                    'z': [0.0, 0.5],
                },
                'grasp_angle': 0.0,
                'retreat_distance': 0.20,
            },
            'control': {
                'gripper_pre_grasp_width': 0.08,
                'gripper_grasp_width': 0.03,
                'gripper_grasp_force': 2.0,
            },
            'behavior': {
                'max_detection_attempts': 5,
                'detection_interval': 0.5,
                'post_grasp_delay': 1.0,
                'place_position': [0.3, 0.3, 0.3],
                'place_orientation': [0.0, 1.0, 0.0, 0.0],
            },
        }

        if config_file and os.path.exists(config_file):
            self.logger.info(f'加载配置文件: {config_file}')
            with open(config_file, 'r', encoding='utf-8') as f:
                user_config = yaml.safe_load(f)
                self._deep_update(default_config, user_config)
        else:
            self.logger.warning('使用默认配置（未指定 -c 或文件不存在）')

        return default_config

    def _deep_update(self, base_dict: dict, update_dict: dict):
        for key, value in update_dict.items():
            if isinstance(value, dict) and key in base_dict \
                    and isinstance(base_dict[key], dict):
                self._deep_update(base_dict[key], value)
            else:
                base_dict[key] = value

    def _create_camera_transform(self) -> CameraToBaseTransform:
        pos = self.config['camera_calibration']['position']
        ori = self.config['camera_calibration']['orientation']

        return CameraToBaseTransform.from_xyz_rpy(
            x=pos[0], y=pos[1], z=pos[2],
            roll=ori[0], pitch=ori[1], yaw=ori[2]
        )

    # ==================================================================
    # 视觉
    # ==================================================================
    def start_vision(self) -> bool:
        if self.is_vision_started:
            self.logger.warning('视觉系统已启动')
            return True

        self.logger.info('启动 RealSense 相机...')
        success = self.vision.start()

        if success:
            self.is_vision_started = True
            self.logger.info('✓ 视觉系统启动成功')
        else:
            self.logger.error('✗ 视觉系统启动失败')

        return success

    def stop_vision(self):
        if self.is_vision_started:
            self.vision.stop()
            self.is_vision_started = False
            self.logger.info('视觉系统已停止')

    def detect_target(self, visualize: bool = True
                      ) -> Optional[Tuple[float, float, float]]:
        if not self.is_vision_started:
            self.logger.error('视觉系统未启动')
            return None

        self.logger.info('检测蓝色纸巾包...')

        max_attempts = self.config['behavior']['max_detection_attempts']
        interval = self.config['behavior']['detection_interval']

        for attempt in range(max_attempts):
            position = self.vision.detect_and_locate(visualize=visualize)

            if position is not None:
                x, y, z = position
                self.logger.info(
                    f'✓ 检测到目标! 相机坐标: ({x:.3f}, {y:.3f}, {z:.3f}) 米'
                )
                self.current_target_position = position
                return position

            self.logger.debug(f'未检测到目标 (尝试 {attempt + 1}/{max_attempts})')
            time.sleep(interval)

        self.logger.warning(f'经过 {max_attempts} 次尝试未检测到目标')
        return None

    # ==================================================================
    # 抓取规划与执行
    # ==================================================================
    def plan_grasp(self, target_position: Tuple[float, float, float]
                   ) -> Optional[GraspPose]:
        x_cam, y_cam, z_cam = target_position

        self.logger.info('规划抓取位姿...')

        grasp_pose = self.planner.compute_grasp_pose(
            x_cam, y_cam, z_cam,
            gripper_width=self.config['grasp']['default_gripper_width'],
            grasp_angle=self.config['grasp']['grasp_angle']
        )

        if grasp_pose is None:
            self.logger.error('✗ 无法规划抓取位姿（目标可能在工作空间外）')
            return None

        self.logger.info('✓ 抓取规划完成')
        return grasp_pose

    def execute_grasp(self, grasp_pose: GraspPose) -> bool:
        self.logger.info('生成抓取轨迹...')

        trajectory = self.planner.generate_grasp_trajectory(grasp_pose)
        if len(trajectory) < 3:
            self.logger.error('轨迹生成失败')
            return False

        approach_pose = trajectory[0]
        grasp_pose = trajectory[1]
        retreat_pose = trajectory[2]

        approach = (approach_pose.position, approach_pose.orientation)
        grasp = (grasp_pose.position, grasp_pose.orientation)
        retreat = (retreat_pose.position, retreat_pose.orientation)

        self.logger.info('执行抓取序列（纯 MoveIt）...')
        success = self.controller.execute_grasp_sequence(
            approach_pose=approach,
            grasp_pose=grasp,
            retreat_pose=retreat,
            pre_grasp_width=self.config['control']['gripper_pre_grasp_width'],
            grasp_width=self.config['control']['gripper_grasp_width'],
            grasp_force=self.config['control']['gripper_grasp_force']
        )

        if success:
            self.logger.info('✓ 抓取成功!')
            time.sleep(self.config['behavior']['post_grasp_delay'])
        else:
            self.logger.error('✗ 抓取失败')

        return success

    def place_object(self) -> bool:
        self.logger.info('移动到放置位置...')

        place_pos = self.config['behavior']['place_position']
        place_ori = self.config['behavior']['place_orientation']

        approach_z = place_pos[2] + 0.1
        success = self.controller.moveit_move_to_pose(
            place_pos[0], place_pos[1], approach_z,
            place_ori[0], place_ori[1], place_ori[2], place_ori[3]
        )
        if not success:
            self.logger.error('移动到放置位置上方失败')
            return False

        time.sleep(0.5)

        self.logger.info('下降到放置高度...')
        success = self.controller.moveit_move_to_pose(
            place_pos[0], place_pos[1], place_pos[2],
            place_ori[0], place_ori[1], place_ori[2], place_ori[3]
        )
        if not success:
            self.logger.error('下降到放置高度失败')
            return False

        time.sleep(0.5)

        self.logger.info('释放物体...')
        self.controller.open_gripper(0.08)
        time.sleep(1.0)

        self.logger.info('撤退...')
        self.controller.moveit_move_to_pose(
            place_pos[0], place_pos[1], approach_z,
            place_ori[0], place_ori[1], place_ori[2], place_ori[3]
        )

        self.logger.info('✓ 物体已放置')
        return True

    # ==================================================================
    # 主循环
    # ==================================================================
    def run_full_cycle(self, visualize: bool = True,
                       place_after_grasp: bool = False) -> bool:
        self.logger.info('\n' + '=' * 70)
        self.logger.info('开始完整抓取循环')
        self.logger.info('=' * 70 + '\n')

        try:
            # 1. 启动视觉系统
            if not self.start_vision():
                return False

            # 2. 回到零位
            self.logger.info('步骤 1: 回到零位')
            self.controller.go_home()
            time.sleep(1.0)

            # 3. 检测目标
            self.logger.info('步骤 2: 检测目标物体')
            target_position = self.detect_target(visualize=visualize)
            if target_position is None:
                self.logger.error('未检测到目标，退出')
                return False

            # 4. 规划抓取
            self.logger.info('步骤 3: 规划抓取')
            grasp_pose = self.plan_grasp(target_position)
            if grasp_pose is None:
                return False

            # 5. 执行抓取
            self.logger.info('步骤 4: 执行抓取')
            success = self.execute_grasp(grasp_pose)
            if not success:
                return False

            # 6. 可选：放置物体
            if place_after_grasp:
                self.logger.info('步骤 5: 放置物体')
                success = self.place_object()
                if not success:
                    self.logger.warning('放置失败，但抓取成功')

            # 7. 回到零位
            self.logger.info('步骤 6: 返回零位')
            self.controller.go_home()

            self.logger.info('\n' + '=' * 70)
            self.logger.info('✓✓✓ 完整抓取循环成功! ✓✓✓')
            self.logger.info('=' * 70 + '\n')
            return True

        except Exception as e:
            self.logger.error(f'抓取循环发生异常: {e}')
            import traceback
            traceback.print_exc()
            return False
        finally:
            self.stop_vision()

    def interactive_mode(self):
        print('\n' + '=' * 70)
        print('蓝色纸巾包视觉抓取系统 - 交互模式（纯 MoveIt）')
        print('=' * 70)

        while True:
            print('\n' + '=' * 70)
            print('请选择操作:')
            print('  1. 运行完整抓取循环（抓取后不放置）')
            print('  2. 运行完整抓取循环（抓取后放置）')
            print('  3. 仅检测目标')
            print('  4. 测试视觉系统')
            print('  5. 回零位')
            print('  6. 测试夹爪')
            print('  7. 重新加载配置')
            print('  8. 测试位姿移动（只位置约束）')
            print('  9. 测试位姿移动（位置+姿态约束）')
            print('  0. 退出')
            print('=' * 70)

            try:
                choice = input('请输入选项 (0-9): ').strip()

                if choice == '0':
                    print('退出系统...')
                    break

                elif choice == '1':
                    self.run_full_cycle(visualize=True, place_after_grasp=False)

                elif choice == '2':
                    self.run_full_cycle(visualize=True, place_after_grasp=True)

                elif choice == '3':
                    if self.start_vision():
                        target = self.detect_target(visualize=True)
                        if target:
                            print(f'检测到目标: {target}')
                        input('按 Enter 继续...')
                        self.stop_vision()

                elif choice == '4':
                    print("启动视觉系统测试（按 'q' 退出）...")
                    if self.start_vision():
                        import cv2
                        while True:
                            self.vision.detect_and_locate(visualize=True)
                            if cv2.waitKey(1) & 0xFF == ord('q'):
                                break
                        self.stop_vision()

                elif choice == '5':
                    self.controller.go_home()

                elif choice == '6':
                    print('测试夹爪...')
                    self.controller.open_gripper(0.08)
                    time.sleep(1.0)
                    self.controller.close_gripper(0.03, 1.5)
                    time.sleep(1.0)
                    self.controller.open_gripper(0.08)

                elif choice == '7':
                    print('重新加载配置...')
                    print('功能待实现')

                elif choice == '8':
                    print('测试位姿移动（只位置约束）')
                    self.controller.use_orientation_constraint = False
                    x = float(input('x (默认 0.379): ').strip() or "0.379")
                    y = float(input('y (默认 0.100): ').strip() or "0.100")
                    z = float(input('z (默认 0.282): ').strip() or "0.282")
                    ok = self.controller.moveit_move_to_pose(x, y, z)
                    print('结果:', '✓ 成功' if ok else '✗ 失败')

                elif choice == '9':
                    print('测试位姿移动（位置+姿态约束）')
                    self.controller.use_orientation_constraint = True
                    x = float(input('x (默认 0.379): ').strip() or "0.379")
                    y = float(input('y (默认 0.100): ').strip() or "0.100")
                    z = float(input('z (默认 0.282): ').strip() or "0.282")
                    qx = float(input('qx (默认 0.0): ').strip() or "0.0")
                    qy = float(input('qy (默认 1.0): ').strip() or "1.0")
                    qz = float(input('qz (默认 0.0): ').strip() or "0.0")
                    qw = float(input('qw (默认 0.0): ').strip() or "0.0")
                    ok = self.controller.moveit_move_to_pose(x, y, z,
                                                                qx, qy, qz, qw)
                    print('结果:', '✓ 成功' if ok else '✗ 失败')

                else:
                    print('无效选项，请重新输入')

            except KeyboardInterrupt:
                print('\n用户中断')
                break
            except Exception as e:
                print(f'发生错误: {e}')
                import traceback
                traceback.print_exc()

    def shutdown(self):
        self.logger.info('关闭系统...')
        self.stop_vision()
        self.controller.shutdown()
        self.logger.info('系统已关闭')


def main():
    import argparse

    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )

    parser = argparse.ArgumentParser(description='蓝色纸巾包视觉抓取系统')
    parser.add_argument('-c', '--config', type=str, default=None,
                        help='配置文件路径')
    parser.add_argument('-a', '--auto', action='store_true',
                        help='自动运行模式（非交互）')
    parser.add_argument('-p', '--place', action='store_true',
                        help='抓取后放置物体')
    parser.add_argument('-v', '--visualize', action='store_true', default=True,
                        help='可视化检测过程')

    args = parser.parse_args()

    rclpy.init()

    try:
        system = BlueTissueGraspSystem(config_file=args.config)

        print('\n' + '=' * 70)
        print('蓝色纸巾包视觉抓取系统（纯 MoveIt 版）')
        print('=' * 70)
        print('确保已启动机械臂控制节点:')
        print('  ros2 launch agx_arm_ctrl start_single_agx_arm_moveit.launch.py \\')
        print('    can_port:=can0 arm_type:=piper effector_type:=agx_gripper')
        print('=' * 70)

        if args.auto:
            print('\n自动运行模式')
            input('按 Enter 键开始...')
            success = system.run_full_cycle(
                visualize=args.visualize,
                place_after_grasp=args.place
            )
            if success:
                print('\n✓ 任务成功完成!')
            else:
                print('\n✗ 任务失败')
        else:
            system.interactive_mode()

    except KeyboardInterrupt:
        print('\n用户中断')
    except Exception as e:
        print(f'\n✗ 发生错误: {e}')
        import traceback
        traceback.print_exc()
    finally:
        if 'system' in locals():
            system.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    easy_handeye2_dir = get_package_share_directory('easy_handeye2')

    handeye_calibrate = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(easy_handeye2_dir, 'launch', 'calibrate.launch.py')
        ),
        launch_arguments={
            'name': 'my_calibration',
            'calibration_type': 'eye_in_hand',
            'tracking_base_frame': 'camera_color_optical_frame',  # 注意
            'tracking_marker_frame': 'aruco_marker_frame',        # 注意
            'robot_base_frame': 'base_link',
            'robot_effector_frame': 'link6',
            'automated': 'true', 
        }.items()
    )

    return LaunchDescription([
        handeye_calibrate,
    ])
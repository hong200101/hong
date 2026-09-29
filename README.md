moveit_demos中：
“”
quick_start为机械臂的快速启动脚本，可选机械臂型号和末端工具
moveit_gripper_demo为带夹爪moveit规划的demo
start_vision_grasp为抓蓝色纸巾的快速启动脚本
“”


眼在手上标定参数：
Translation
	x: -0.070701
	y: -0.000912
	z: 0.037813)
Rotation
	x: -0.133285
	y: 0.126056
	z: -0.659821
	w: 0.728684



手眼标定需要aruco_ros和easy_handeye2俩个包进行
重点部分：
handeye_calibrate.launch.py中需要重点对应的frame：'tracking_base_frame': 'camera_color_optical_frame',    'tracking_marker_frame': 'aruco_marker_frame', 
ros2的easy_handeye2没有自动位姿采样，建议示教模式拖动机械臂采样，或者使用auto_move代码自动生成位姿

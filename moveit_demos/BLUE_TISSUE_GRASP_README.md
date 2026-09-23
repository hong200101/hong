# 🤖 Blue Tissue Package Visual Grasping System

A complete visual grasping solution for Piper robotic arm with RealSense D435 camera, featuring MoveIt motion planning and MIT compliant control.

## ✨ Features

- 🎯 **Intelligent Vision**: HSV-based blue object detection with 3D localization
- 📐 **Precise Planning**: Camera-to-base coordinate transformation and grasp pose calculation
- 🎮 **Hybrid Control**: MoveIt coarse positioning + MIT compliant fine-tuning
- 🤲 **Adaptive Grasping**: Automatic trajectory generation and execution
- 🛡️ **Safe & Reliable**: Workspace limits, collision detection, and compliant control

## 📦 System Components

```
blue_tissue_grasp.py       # Main program (system integration)
├── vision_module.py       # Vision module (camera + detection)
├── grasp_planner.py      # Planning module (pose calculation)
└── hybrid_controller.py  # Control module (MoveIt + MIT)
```

## 🔧 Requirements

### Hardware
- **Robot Arm**: Piper 6-DOF (with MIT mode firmware)
- **End Effector**: AgileX Gripper (0-100mm opening)
- **Camera**: Intel RealSense D435
- **CAN Module**: Official CAN converter (1Mbps)
- **Computer**: Ubuntu 22.04 with ROS2 Humble/Jazzy

### Software Dependencies
```bash
# Python packages
pip3 install pyrealsense2 opencv-python numpy pyyaml transforms3d

# ROS2 packages
sudo apt install ros-$ROS_DISTRO-realsense2-*
sudo apt install ros-$ROS_DISTRO-moveit
```

## 🚀 Quick Start

### 1. Start Robot Control Node

**Terminal 1**:
```bash
cd ~/agx_arm_ws
source install/setup.bash

ros2 launch agx_arm_ctrl start_single_agx_arm_moveit.launch.py \
  can_port:=can0 \
  arm_type:=piper \
  effector_type:=agx_gripper
```

### 2. Run Grasping System

**Terminal 2**:
```bash
cd ~/agx_arm_ws/src/agx_arm_ros/moveit_demos

# Interactive mode
python3 blue_tissue_grasp.py

# Auto mode (single grasp)
python3 blue_tissue_grasp.py --auto

# Auto mode with placement
python3 blue_tissue_grasp.py --auto --place
```

## 📋 Workflow

```
Vision Detection → Coordinate Transform → Grasp Planning → 
MoveIt Approach → MIT Compliant Descent → Gripper Grasp → Retreat
```

## ⚙️ Configuration

Edit `config/blue_tissue_grasp_config.yaml`:

```yaml
# Vision settings
vision:
  blue_hsv_lower: [90, 50, 50]    # Adjust for your lighting
  blue_hsv_upper: [130, 255, 255]
  min_area: 500

# Camera calibration (IMPORTANT: calibrate for your setup!)
camera_calibration:
  position: [0.4, 0.0, 0.5]        # Camera position in base frame
  orientation: [0.0, 3.14159, 0.0] # Camera pointing down

# MIT control parameters
control:
  mit_params:
    compliant:
      kp: 20.0  # Position gain (stiffness)
      kd: 1.0   # Velocity gain (damping)
```

## 🎮 Interactive Mode Menu

```
1. Run full grasp cycle (without placement)
2. Run full grasp cycle (with placement)
3. Detect target only
4. Test vision system
5. Go to home position
6. Test gripper
0. Exit
```

## 🔍 Testing Individual Modules

```bash
# Test vision module
python3 vision_module.py

# Test grasp planner
python3 grasp_planner.py

# Test hybrid controller
python3 hybrid_controller.py
```

## 📝 Camera Calibration

**Critical Step**: You must calibrate the camera-to-base transformation!

### Method 1: Manual Measurement (Quick)
1. Measure camera position in base frame (x, y, z)
2. Measure camera orientation (typically pitch = π for downward)
3. Update `camera_calibration` in config file

### Method 2: ArUco Board Calibration (Accurate)
```bash
# Install easy_handeye
sudo apt install ros-$ROS_DISTRO-easy-handeye

# Run calibration
ros2 launch easy_handeye calibrate.launch.py
```

## 🐛 Troubleshooting

### Camera not detected
```bash
# Check camera connection
rs-enumerate-devices

# Fix permissions
sudo chmod 777 /dev/video*
```

### Blue object not detected
- Adjust HSV range using color calibration tool (press 'c' in vision test)
- Improve lighting conditions
- Lower `min_area` threshold

### Grasp position offset
- Re-calibrate camera-to-base transform
- Adjust `grasp_depth_offset` in config

### MIT mode oscillation
- Decrease `kp` (position gain)
- Increase `kd` (velocity gain)

## 📊 Parameter Tuning Guide

### Vision Parameters
- **Light too bright**: Increase `V_min` (e.g., 100)
- **Light too dim**: Decrease `V_min` (e.g., 30)
- **Too many false detections**: Increase `min_area`

### MIT Control Parameters
- **Oscillation**: Reduce `kp`, increase `kd`
- **Too slow**: Increase `kp`
- **Not compliant enough**: Reduce `kp` to 5-10

### Grasp Planning
- **Target unreachable**: Expand `workspace_limits`
- **Grasp too high**: Increase `grasp_depth_offset` (more negative)
- **Grasp too low**: Decrease `grasp_depth_offset`

## 📚 File Structure

```
moveit_demos/
├── blue_tissue_grasp.py              # Main program
├── vision_module.py                  # Vision processing
├── grasp_planner.py                  # Grasp planning
├── hybrid_controller.py              # Hybrid control
├── config/
│   └── blue_tissue_grasp_config.yaml # Configuration
├── run_blue_tissue_grasp.sh          # Launch script
├── requirements.txt                  # Python dependencies
├── 蓝色纸巾包视觉抓取使用指南.md       # Chinese guide
└── BLUE_TISSUE_GRASP_README.md       # This file
```

## 🎯 Usage Examples

### Example 1: Auto grasp once
```bash
python3 blue_tissue_grasp.py --auto
```

### Example 2: Auto grasp and place
```bash
python3 blue_tissue_grasp.py --auto --place
```

### Example 3: Use custom config
```bash
python3 blue_tissue_grasp.py --config my_config.yaml
```

### Example 4: Interactive mode
```bash
python3 blue_tissue_grasp.py
# Then select menu options
```

## 🔬 Advanced Features

### Multi-object Grasping
Modify main loop to grasp multiple objects:
```python
for i in range(3):
    system.run_full_cycle(place_after_grasp=True)
```

### Custom Grasp Strategy
Edit `grasp_planner.py` to implement adaptive gripper width:
```python
width, height = self.estimate_object_size(...)
gripper_width = min(width + 0.02, 0.10)
```

### ROS2 Service Integration
Convert to ROS2 service for remote control:
```python
self.srv = self.create_service(Trigger, 'execute_grasp', callback)
```

## 📞 Support

- **GitHub Issues**: https://github.com/agilexrobotics/agx_arm_ros/issues
- **Email**: support@agilex.ai
- **Documentation**: https://github.com/agilexrobotics/agx_arm_ros

## 📄 License

MIT License

---

<div align="center">
  <p><strong>Happy Grasping!</strong></p>
  <p>For detailed Chinese documentation, see: 蓝色纸巾包视觉抓取使用指南.md</p>
</div>

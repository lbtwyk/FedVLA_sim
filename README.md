# myCobot Cube Stacking with Visuomotor Diffusion Policy

A ROS 2 workspace for cube stacking simulation using the myCobot 280 robot arm. This project combines traditional motion planning with visuomotor diffusion policy inference for robotic manipulation tasks.

## Overview

This workspace implements a complete pipeline for:

1. **Traditional Motion Planning**: MoveIt 2-based deterministic cube stacking
2. **Data Collection**: Automated trajectory recording for machine learning training
3. **Diffusion Policy Training**: Neural network training for visuomotor control
4. **Model Inference**: Real-time robot control using trained diffusion policies

### Key Features

- **Simulation Environment**: Gazebo Garden with enhanced physics and friction
- **Robot Control**: myCobot 280 6-DOF arm with gripper control
- **Vision System**: 424x240 camera feed for visuomotor learning
- **Data Pipeline**: Synchronized joint states, gripper positions, and camera images
- **Machine Learning**: PyTorch-based diffusion policy training and inference
- **Robust Planning**: Retry mechanisms and fallback strategies

## System Requirements

- **OS**: Ubuntu 24.04 (WSL2 supported)
- **ROS**: ROS 2 Jazzy Jalisco
- **Simulator**: Gazebo Garden
- **GPU**: NVIDIA GPU with CUDA support (recommended)
- **Python**: 3.10+ with PyTorch

## Project Structure

```
.
├── src/                             # ROS 2 source packages
│   ├── mycobot_stacking_project/    # Main stacking project package
│   │   ├── launch/                  # Launch files
│   │   │   ├── fixed_stacking_task.launch.py # Traditional motion planning
│   │   │   ├── collect_data.launch.py        # Data collection for training
│   │   │   └── mycobot_gazebo_with_friction.launch.py # Gazebo simulation
│   │   ├── src/                     # C++ source files
│   │   │   ├── cube_spawner_node.cpp         # Cube spawning and randomization
│   │   │   └── stacking_manager_node.cpp     # Motion planning and control
│   │   ├── worlds/                  # Gazebo world files
│   │   ├── models/                  # Cube models (yellow/orange)
│   │   └── urdf/                    # Robot descriptions with friction
│   ├── diffusion_policy_inference/  # Model inference package
│   │   ├── diffusion_policy_inference/       # Python inference nodes
│   │   ├── launch/                           # Inference launch files
│   │   └── scripts/                          # Utility scripts
│   ├── trajectory_data_collector/   # Data collection system
│   │   └── trajectory_data_collector/        # State logging implementation
│   ├── trajectory_data_interfaces/  # Custom ROS 2 interfaces
│   │   └── srv/                              # Service definitions
│   └── mycobot_ros2/               # Robot description and configuration
│       ├── mycobot_description/              # URDF and meshes
│       ├── mycobot_gazebo/                   # Gazebo integration
│       └── mycobot_moveit_config/            # MoveIt configuration
├── DP/                             # Diffusion policy training system
│   ├── model.py                    # Neural network architecture
│   ├── train.py                    # Training script
│   ├── dataset.py                  # Data loading and preprocessing
│   ├── inference*.py               # Various inference implementations
│   ├── requirements.txt            # Python dependencies
│   └── checkpoints/                # Trained model files
├── mycobot_episodes_degrees/       # Collected training data
├── collect_multiple_episodes.sh    # Automated data collection script
├── launch_cube_stacking.sh        # Convenience launch script
├── run_inference_visualization.sh  # Inference visualization script
└── check_episodes.py              # Episode management and validation tool
```

## Package Documentation

### Core Packages
- **[myCobot Stacking Project](src/mycobot_stacking_project/README.md)** - Main simulation and motion planning
- **[Diffusion Policy Inference](src/diffusion_policy_inference/README.md)** - Model inference and robot control
- **[Trajectory Data Collector](src/trajectory_data_collector/README.md)** - Data collection system
- **[Trajectory Data Interfaces](src/trajectory_data_interfaces/README.md)** - Custom ROS 2 interfaces

### Training and Models
- **[Diffusion Policy Training](DP/README.md)** - Model training and architecture
- **[Data Collection Guide](DATA_COLLECTION.md)** - Detailed data collection documentation

## Key Components

### Cube Stacking System
- **Cube Dimensions**: 2.5cm x 2.5cm x 2.5cm cubes (yellow and orange)
- **Cube Positioning**: Yellow at (0, 0.20), Orange at (0.035, 0.25) in world coordinates
- **Heights**: World cubes at z=0.05m, planning scene at z=0.0125m
- **Gripper Control**: Position range from -0.5 (closed) to 0.0 (open)

### Motion Planning
- **Movement Heights**: Pre-grasp z=0.15m, Grasp z=0.11m, Lift z=0.15m, Place z=0.14m
- **Planning**: MoveIt 2 with Cartesian path planning and retry mechanisms
- **Speed**: 75% of maximum velocity for stability
- **Timing**: 0.2s confirmation times with immediate retry on failure

### Diffusion Policy Model
- **Vision**: ResNet34 backbone for 424x240 camera images
- **Output**: 7-dimensional action (6 joints + 1 gripper)
- **Training Data**: Degrees format for better model sensitivity
- **Inference**: 10Hz real-time control

## Installation

### 1. ROS 2 Workspace Setup

1. Create a ROS 2 workspace:
   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws/src
   ```

2. Clone the repository:
   ```bash
   git clone https://github.com/lbtwyk/FedVLA_sim.git .
   ```

3. Install ROS 2 dependencies:
   ```bash
   cd ~/ros2_ws
   rosdep install --from-paths src --ignore-src -r -y
   ```

4. Build the workspace:
   ```bash
   colcon build --symlink-install
   ```

5. Source the workspace:
   ```bash
   source ~/ros2_ws/install/setup.bash
   ```

### 2. Python Virtual Environment Setup

For diffusion policy training and inference:

```bash
# Create and activate virtual environment
python3 -m venv ~/.venvs/diffusion_policy
source ~/.venvs/diffusion_policy/bin/activate

# Install dependencies
cd ~/ros2_ws/DP
pip install -r requirements.txt
```

## Usage

### Quick Start

Use the convenience script for easy launching:
```bash
cd ~/ros2_ws
./launch_cube_stacking.sh
```

### Manual Launch Options

#### 1. Traditional Motion Planning

Launch the cube stacking simulation with MoveIt 2:
```bash
# Clean up any existing processes
pkill -9 -f "ros2|gazebo|gz|rviz2|robot_state_publisher|move_group|cube_spawner|stacking_manager"

# Launch simulation
ros2 launch mycobot_stacking_project fixed_stacking_task.launch.py
```

#### 2. Data Collection for Training

Collect trajectory data for diffusion policy training:
```bash
# Single episode
ros2 launch mycobot_stacking_project collect_data.launch.py output_base_dir:=~/mycobot_episodes_degrees

# Multiple episodes (automated)
./collect_multiple_episodes.sh
```

#### 3. Diffusion Policy Inference

Run inference with a trained model:
```bash
# Activate virtual environment
source ~/.venvs/diffusion_policy/bin/activate

# Run inference visualization
./run_inference_visualization.sh
```

### Episode Management

Check and manage collected episodes:
```bash
# Check episode status
python3 check_episodes.py scan --dir ~/mycobot_episodes_degrees

# Clean up failed episodes
python3 check_episodes.py clean
```

## Configuration

### Key Parameters
- **Cube Positions**: Yellow (0, 0.20), Orange (0.035, 0.25)
- **Cube Heights**: World z=0.05m, Planning scene z=0.0125m
- **Gripper Range**: -0.5 (closed) to 0.0 (open)
- **Movement Heights**: Pre-grasp 0.15m, Grasp 0.11m, Lift 0.15m, Place 0.14m
- **Data Collection**: 10Hz frequency, degrees format
- **Camera**: 424x240 resolution at `/camera_head/color/image_raw`

## Troubleshooting

### Common Issues
- **Launch Errors**: Always run `pkill -9 -f "ros2|gazebo|gz|rviz2|robot_state_publisher|move_group|cube_spawner|stacking_manager"` before launching
- **Planning Failures**: Rebuild workspace with `colcon build` before launching
- **Model Loading**: Ensure checkpoint exists and virtual environment is activated
- **Data Collection**: Episodes save to `~/mycobot_episodes_degrees/` in degrees format

### Debug Commands
```bash
# Check topics
ros2 topic list | grep -E 'joint_states|camera|move_group'

# Validate episodes
python3 check_episodes.py scan --dir ~/mycobot_episodes_degrees

# Check virtual environment
echo $VIRTUAL_ENV
```

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Author

Yukun Wang (lambert.wang.23@ucl.ac.uk)

## Acknowledgments

This project uses the myCobot ROS 2 packages developed by Automatic Addison. The base robot packages and simulation components are adapted from the [mycobot_ros2](https://github.com/automaticaddison/mycobot_ros2) repository.

**Copyright Notice**: The myCobot ROS 2 base packages are copyright © Automatic Addison and are used under their respective license terms.

---

## Related Documentation

- **[Data Collection Guide](DATA_COLLECTION.md)** - Comprehensive data collection documentation
- **[Diffusion Policy Training](DP/README.md)** - Model training and architecture details
- **[Episode Management Tool](docs/episode_management_tool.md)** - Episode validation and cleanup tools

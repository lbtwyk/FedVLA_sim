# FedVLA_sim - Cube Stacking with Diffusion Policy

A comprehensive ROS 2 simulation project for robotic cube stacking using the myCobot 280 robot arm in Gazebo. This project demonstrates both traditional motion planning and modern diffusion policy-based control for precise manipulation tasks.

## Overview

This project implements a complete cube stacking pipeline with multiple control approaches:

### Traditional Motion Planning
1. Spawns colored cubes (yellow and orange) in a Gazebo simulation
2. Adds the cubes to the MoveIt planning scene
3. Controls the myCobot 280 robot arm using MoveIt 2 to:
   - Locate the yellow cube
   - Pick it up with the gripper
   - Move it to the orange cube
   - Place it precisely on top
   - Return to the home position

### Diffusion Policy Control
1. Collects trajectory data from traditional motion planning
2. Trains a visuomotor diffusion policy model using camera images and robot states
3. Performs inference using the trained model to control the robot directly
4. Provides visualization tools to evaluate model performance

The project uses ROS 2 Jazzy Jalisco, MoveIt 2 for motion planning, and PyTorch for diffusion policy training and inference.

## System Requirements

- Ubuntu 24.04 (can run in WSL2 with GPU acceleration)
- ROS 2 Jazzy Jalisco
- Gazebo Garden
- MoveIt 2
- Python 3.10+ with PyTorch for diffusion policy
- NVIDIA GPU with WSL2 drivers (for optimal performance)

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
│   │   └── rviz/                    # RViz configurations
│   ├── diffusion_policy_inference/  # Diffusion policy inference package
│   │   ├── diffusion_policy_inference/      # Python inference node
│   │   ├── launch/                  # Inference launch files
│   │   └── scripts/                 # Convenience scripts
│   ├── trajectory_data_collector/   # Data collection system
│   │   └── trajectory_data_collector/       # State logging and episode management
│   ├── trajectory_data_interfaces/  # Custom ROS 2 interfaces
│   │   └── srv/                     # Service definitions for data collection
│   └── mycobot_ros2/               # Base myCobot ROS 2 packages
│       ├── mycobot_description/     # Robot URDF and meshes
│       ├── mycobot_gazebo/         # Gazebo integration
│       ├── mycobot_moveit_config/  # MoveIt configuration
│       └── ...                     # Additional myCobot packages
├── DP/                             # Diffusion Policy training system
│   ├── model.py                    # Neural network architecture (ResNet34 + MLP)
│   ├── train.py                    # Training script with diffusion sampling
│   ├── dataset.py                  # Data loading and preprocessing
│   ├── checkpoints/                # Trained model checkpoints
│   └── requirements.txt            # Python dependencies
├── mycobot_episodes/               # Collected trajectory data (radians)
├── mycobot_episodes_degrees/       # Collected trajectory data (degrees)
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

### Base Robot Support
- **[myCobot ROS 2](src/mycobot_ros2/README.md)** - Base robot packages and simulation

## Key Components

### Stacking Manager Node
The `StackingManagerNode` implements traditional motion planning logic:
- Waits for cubes to be detected in the planning scene
- Executes pick-and-place operations using MoveIt 2
- Supports both Cartesian and joint-space planning
- Includes robust error handling and retry mechanisms

### Diffusion Policy Model
The diffusion policy system includes:
- **ResNet34 backbone** for visual feature extraction from 424x240 camera images
- **MLP network** for state prediction with 6 joint angles + 1 gripper value
- **Diffusion sampling** for robust action generation
- **Real-time inference** at 10Hz for smooth robot control

### Data Collection System
Automated trajectory data collection with:
- Synchronized camera images and robot states
- Configurable recording frequency (default: 10Hz)
- Episode management and validation tools
- Support for both radians and degrees joint representations

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

For diffusion policy training and inference, set up a Python virtual environment:

1. Create and activate virtual environment:
   ```bash
   python3 -m venv ~/.venvs/diffusion_policy
   source ~/.venvs/diffusion_policy/bin/activate
   ```

2. Install Python dependencies:
   ```bash
   cd ~/ros2_ws/DP
   pip install -r requirements.txt
   ```

3. Or use the provided setup script:
   ```bash
   cd ~/ros2_ws/src/diffusion_policy_inference
   ./setup_venv.sh
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
ros2 launch mycobot_stacking_project collect_data.launch.py output_base_dir:=~/mycobot_episodes

# Multiple episodes (automated)
./collect_multiple_episodes.sh --num-episodes 50 --output-dir ~/mycobot_episodes_degrees
```

#### 3. Diffusion Policy Inference

Run inference with a trained model:
```bash
# Activate virtual environment
source ~/.venvs/diffusion_policy/bin/activate

# Run inference visualization
./run_inference_visualization.sh

# Or run inference directly
ros2 launch diffusion_policy_inference simulation_inference.launch.py \
  checkpoint_path:=/home/wangyukun/ros2_ws/DP/checkpoints/model_best-2.pth \
  model_dir:=/home/wangyukun/ros2_ws/DP
```

### Episode Management

Check and manage collected episodes:
```bash
# Check episode status
python3 check_episodes.py scan

# Clean up failed episodes
python3 check_episodes.py clean
```

## Key Features

### Simulation and Control
- **Enhanced Friction**: Increased friction parameters to prevent cubes from slipping during manipulation
- **Robust Motion Planning**: Uses MoveIt's planning capabilities with increased planning time and attempts
- **Dual Control Modes**: Traditional motion planning and modern diffusion policy control
- **Real-time Performance**: 10Hz inference rate for smooth robot control

### Data Collection and Training
- **Automated Data Collection**: Multi-episode collection with randomized cube positions
- **Synchronized Recording**: Camera images (424x240) and robot states at 10Hz
- **Episode Validation**: Automatic detection and cleanup of failed episodes
- **Flexible Data Formats**: Support for both radians and degrees joint representations

### Machine Learning
- **ResNet34 Vision Backbone**: Pre-trained feature extraction for robust visual processing
- **Diffusion Policy**: State-of-the-art generative model for robot control
- **GPU Acceleration**: CUDA support for training and inference
- **Model Checkpointing**: Automatic saving of best models during training

## Configuration

### Camera Settings
- **Resolution**: 424x240 pixels (optimized for diffusion policy)
- **Frame Rate**: 10Hz for data collection and inference
- **Topic**: `/camera_head/color/image_raw`

### Robot Configuration
- **Joints**: 6 DOF arm + 1 gripper (7 total states)
- **Gripper Range**: 0 (closed) to 100 (open)
- **Planning Algorithm**: OMPL with Cartesian path planning
- **Speed**: 75% of maximum for stable operation

### Model Paths
- **Trained Model**: `/home/wangyukun/ros2_ws/DP/checkpoints/model_best-2.pth`
- **Training Data**: `~/mycobot_episodes_degrees/` (degrees format recommended)
- **Virtual Environment**: `~/.venvs/diffusion_policy`

## Troubleshooting

### Common Issues
- **Cube Slipping**: Check friction parameters in URDF and world files
- **Planning Failures**: Increase planning time and attempts in stacking manager
- **Model Loading Errors**: Ensure checkpoint file exists and virtual environment is activated
- **Launch Errors**: Always terminate existing processes before launching new ones

### Debug Commands
```bash
# Check ROS 2 topics
ros2 topic list | grep -E 'joint_states|camera|move_group'

# Monitor episode collection
tail -f /tmp/episode_output.log

# Check virtual environment
echo $VIRTUAL_ENV

# Validate episodes
python3 check_episodes.py scan --dir ~/mycobot_episodes_degrees
```

## Development Notes

- **Build Requirement**: Rebuild workspace after code changes: `colcon build --symlink-install`
- **Cube Dimensions**: 2.5cm cubes with randomized positions
- **Timing**: 10-second Gazebo startup delay, 45-second stacking manager delay
- **Testing**: Use RViz motion planning panel for interactive parameter adjustment

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Author

Yukun Wang (lambert.wang.23@ucl.ac.uk)

---

## Related Documentation

- **[Data Collection Guide](DATA_COLLECTION.md)** - Comprehensive data collection documentation
- **[Diffusion Policy Training](DP/README.md)** - Model training and architecture details
- **[Episode Management Tool](docs/episode_management_tool.md)** - Episode validation and cleanup tools

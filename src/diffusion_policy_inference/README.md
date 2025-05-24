# Diffusion Policy Inference for Cube Stacking

A ROS 2 package for running inference with trained diffusion policy models in a simulated environment. This package enables real-time robot control using visuomotor diffusion policies for cube stacking tasks with the myCobot 280 robot.

## Overview

The package provides a complete inference pipeline:

### Core Components

1. **Model Inference Node** (`model_inference_node`):
   - Loads trained diffusion policy models with ResNet34 backbone
   - Subscribes to 424x240 camera images from simulation
   - Runs real-time inference at 10Hz
   - Publishes joint commands to robot controllers
   - Supports GPU acceleration for fast inference

2. **Simulation Launch** (`simulation_inference.launch.py`):
   - Starts Gazebo with cube stacking world
   - Launches MoveIt 2 for robot planning scene
   - Spawns randomized cube positions
   - Initializes camera and robot controllers
   - Starts inference node with model loading

3. **Convenience Scripts**:
   - `run_simulation_inference.sh`: Complete simulation setup
   - `setup_venv.sh`: Virtual environment configuration
   - Wrapper scripts for easy execution

### Key Features

- **Real-time Performance**: 10Hz inference rate for smooth control
- **ResNet34 Vision**: Pre-trained backbone for robust visual processing
- **Diffusion Sampling**: DDPM-based action generation
- **GPU Acceleration**: CUDA support for fast inference
- **Model Flexibility**: Support for different checkpoint formats

## Prerequisites

### System Requirements
- ROS 2 Jazzy Jalisco
- Gazebo Garden
- Python 3.10+ with PyTorch
- NVIDIA GPU with CUDA support (recommended)
- Trained diffusion policy model checkpoint

### Model Requirements
- **Checkpoint Format**: PyTorch .pth file with model_state_dict and args
- **Architecture**: ResNet34 + MLP (7-dimensional output)
- **Input Resolution**: 424x240 RGB images
- **Training Data**: Preferably degrees format for better sensitivity

## Installation

### 1. ROS 2 Package Setup

The package is included in the main workspace. Ensure it's built:

```bash
cd ~/ros2_ws
colcon build --packages-select diffusion_policy_inference
source install/setup.bash
```

### 2. Python Virtual Environment

Set up a dedicated virtual environment for diffusion policy:

```bash
# Create virtual environment
python3 -m venv ~/.venvs/diffusion_policy
source ~/.venvs/diffusion_policy/bin/activate

# Install dependencies
cd ~/ros2_ws/DP
pip install -r requirements.txt
```

**Or use the provided setup script**:

```bash
cd ~/ros2_ws/src/diffusion_policy_inference
./setup_venv.sh
```

### 3. Model Preparation

Ensure you have a trained model checkpoint:

```bash
# Check model exists
ls -la ~/ros2_ws/DP/checkpoints/model_best-2.pth

# Verify model directory structure
ls -la ~/ros2_ws/DP/
# Should contain: model.py, train.py, dataset.py
```

## Usage

### Quick Start

**Recommended approach using the visualization script**:

```bash
# Activate virtual environment
source ~/.venvs/diffusion_policy/bin/activate

# Run inference visualization
cd ~/ros2_ws
./run_inference_visualization.sh
```

This script automatically:
- Uses the default model checkpoint (`model_best-2.pth`)
- Sets up proper camera and simulation parameters
- Provides real-time visualization of model predictions

### Manual Launch Options

#### 1. Using Convenience Script

```bash
cd ~/ros2_ws
source ~/.venvs/diffusion_policy/bin/activate

./src/diffusion_policy_inference/run_simulation_inference.sh \
  --checkpoint ~/ros2_ws/DP/checkpoints/model_best-2.pth \
  --model-dir ~/ros2_ws/DP \
  --rate 10.0 \
  --log-level info
```

**Parameters**:
- `--checkpoint`: Path to trained model checkpoint (required)
- `--model-dir`: Directory containing model.py and train.py (required)
- `--rate`: Inference rate in Hz (default: 10.0)
- `--log-level`: ROS log level (default: info)

#### 2. Direct ROS 2 Launch

```bash
source ~/.venvs/diffusion_policy/bin/activate

ros2 launch diffusion_policy_inference simulation_inference.launch.py \
  checkpoint_path:=~/ros2_ws/DP/checkpoints/model_best-2.pth \
  model_dir:=~/ros2_ws/DP \
  inference_rate:=10.0 \
  log_level:=info
```

#### 3. Using Virtual Environment Wrapper

```bash
# If you used setup_venv.sh
~/ros2_ws/run_inference_with_venv.sh \
  --checkpoint ~/ros2_ws/DP/checkpoints/model_best-2.pth \
  --model-dir ~/ros2_ws/DP
```

## Model Configuration

### Checkpoint Requirements

The inference node expects a PyTorch checkpoint file containing:

```python
checkpoint = {
    'model_state_dict': model.state_dict(),    # Trained model weights
    'args': {                                  # Training configuration
        'state_dim': 7,                        # 6 joints + 1 gripper
        'hidden_dim': 256,                     # MLP hidden dimension
        'num_mlp_layers': 4,                   # MLP depth
        'image_feature_dim': 512,              # ResNet34 output dim
        'timesteps': 1000,                     # Diffusion timesteps
        'beta_start': 1e-4,                    # Beta schedule start
        'beta_end': 0.02,                      # Beta schedule end
        # ... other training parameters
    }
}
```

### Model Directory Structure

The `model_dir` must contain:

```
~/ros2_ws/DP/
├── model.py                    # DiffusionPolicyModel class
├── train.py                    # Diffusion sampling functions
├── dataset.py                  # Data loading utilities
└── checkpoints/
    └── model_best-2.pth        # Trained checkpoint
```

**Required Functions in train.py**:
- `linear_beta_schedule()`: Beta schedule generation
- `extract()`: Tensor extraction utility
- `p_sample()`: Single diffusion step
- `p_sample_loop()`: Complete sampling loop

## Node Configuration

### Parameters

**Core Parameters**:
- `checkpoint_path` (string): Path to model checkpoint file
- `model_dir` (string): Directory containing model.py and train.py
- `inference_rate` (double): Inference frequency in Hz (default: 10.0)
- `device` (string): Computation device - "cuda" or "cpu" (auto-detected)

**Topic Configuration**:
- `image_topic` (string): Camera image topic (default: "/camera_head/color/image_raw")
- `joint_states_topic` (string): Joint command topic (default: "/joint_states")
- `joint_command_topic` (string): Joint command output (default: "/joint_group_position_controller/commands")

**Robot Configuration**:
- `joint_names` (string[]): Robot joint names for command publishing

### Custom Node Launch

```bash
ros2 run diffusion_policy_inference model_inference_node \
  --ros-args \
  -p checkpoint_path:=~/ros2_ws/DP/checkpoints/model_best-2.pth \
  -p model_dir:=~/ros2_ws/DP \
  -p inference_rate:=10.0 \
  -p device:=cuda \
  -p image_topic:=/camera_head/color/image_raw
```

## Performance Optimization

### GPU Acceleration

```bash
# Check CUDA availability
python3 -c "import torch; print(f'CUDA available: {torch.cuda.is_available()}')"

# Monitor GPU usage during inference
nvidia-smi -l 1
```

### Inference Timing

The system is optimized for real-time performance:
- **Target Rate**: 10Hz (100ms per inference)
- **Typical Performance**: 50-80ms on RTX 3070
- **Bottlenecks**: Image preprocessing and diffusion sampling

### Memory Usage

- **Model Size**: ~25MB (ResNet34 + MLP)
- **GPU Memory**: ~2GB during inference
- **CPU Memory**: ~1GB for image processing

## Troubleshooting

### Common Issues

**1. Virtual Environment Not Activated**:
```bash
# Check current environment
echo $VIRTUAL_ENV

# Should show: /home/username/.venvs/diffusion_policy
# If not, activate:
source ~/.venvs/diffusion_policy/bin/activate
```

**2. Model Loading Errors**:
```bash
# Check checkpoint file
python3 -c "
import torch
checkpoint = torch.load('~/ros2_ws/DP/checkpoints/model_best-2.pth', map_location='cpu')
print('Keys:', checkpoint.keys())
print('Args:', checkpoint.get('args', 'Missing args'))
"
```

**3. CUDA Issues**:
```bash
# Check CUDA installation
nvidia-smi
python3 -c "import torch; print(torch.version.cuda)"

# Force CPU mode if needed
ros2 run diffusion_policy_inference model_inference_node \
  --ros-args -p device:=cpu
```

**4. Topic Connection Issues**:
```bash
# Check available topics
ros2 topic list | grep -E 'camera|joint'

# Monitor image topic
ros2 topic hz /camera_head/color/image_raw

# Check joint commands
ros2 topic echo /joint_group_position_controller/commands
```

### Debug Commands

```bash
# Monitor inference node
ros2 node info /model_inference_node

# Check node parameters
ros2 param list /model_inference_node

# View inference logs
ros2 log view | grep model_inference

# Test model loading
cd ~/ros2_ws/DP
python3 -c "
from model import DiffusionPolicyModel
model = DiffusionPolicyModel(state_dim=7)
print('Model loaded successfully')
"
```

### Performance Monitoring

```bash
# Monitor system resources
htop

# Check inference timing
ros2 topic hz /joint_group_position_controller/commands

# Monitor GPU usage
watch -n 1 nvidia-smi
```

## Integration Examples

### Custom Robot Configuration

```python
# Custom joint names for different robots
joint_names = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint"
]
```

### Alternative Camera Topics

```bash
# Different camera configurations
ros2 launch diffusion_policy_inference simulation_inference.launch.py \
  image_topic:=/front_camera/image_raw \
  checkpoint_path:=~/models/custom_model.pth
```

## Dependencies

**ROS 2 Packages**:
- rclpy
- sensor_msgs
- std_msgs
- cv_bridge

**Python Packages**:
- torch >= 1.12.0
- torchvision >= 0.13.0
- Pillow >= 8.0.0
- opencv-python >= 4.5.0
- numpy >= 1.21.0

## Related Documentation

- **[Diffusion Policy Training](../../DP/README.md)** - Model training and architecture
- **[myCobot Stacking Project](../mycobot_stacking_project/README.md)** - Simulation environment
- **[Main Project README](../../README.md)** - Overall project documentation

## Back to Main Documentation

← [Main Project README](../../README.md)

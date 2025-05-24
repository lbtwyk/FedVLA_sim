# Diffusion Policy Inference for Cube Stacking

This package provides tools for running inference with a trained diffusion policy model in a simulated environment. It allows you to use your trained model to control a simulated myCobot 280 robot for cube stacking tasks.

## Overview

The package includes:

1. A ROS 2 node (`model_inference_node`) that:
   - Loads a trained diffusion policy model from a checkpoint
   - Subscribes to camera images from the simulation
   - Runs inference on those images
   - Sends control commands to the simulated robot

2. A launch file (`simulation_inference.launch.py`) that:
   - Starts Gazebo with the cube stacking world
   - Launches MoveIt for the myCobot 280 robot
   - Spawns cubes in the simulation
   - Starts the model inference node

3. A convenience script (`run_simulation_inference.sh`) for running the simulation with inference

## Prerequisites

1. ROS 2 Jazzy Jalisco
2. Gazebo Sim
3. Python 3 with PyTorch
4. Trained diffusion policy model checkpoint

## Installation

1. Clone this repository into your ROS 2 workspace:
   ```bash
   cd ~/ros2_ws/src
   ```

2. Install Python dependencies:
   ```bash
   pip3 install torch torchvision pillow opencv-python
   ```

3. Build the workspace:
   ```bash
   cd ~/ros2_ws
   colcon build --symlink-install
   ```

4. Source the workspace:
   ```bash
   source ~/ros2_ws/install/setup.bash
   ```

## Usage

### Running Simulation Inference

Use the provided script to run simulation inference:

```bash
cd ~/ros2_ws
./src/diffusion_policy_inference/run_simulation_inference.sh \
  --checkpoint /path/to/your/model_best.pth \
  --model-dir /path/to/your/model/directory \
  --rate 10.0 \
  --log-level info
```

Parameters:
- `--checkpoint`: Path to your trained model checkpoint file (required)
- `--model-dir`: Directory containing `model.py` and `train.py` (required)
- `--rate`: Inference rate in Hz (default: 10.0)
- `--log-level`: ROS log level (default: info)

### Manual Launch

You can also launch the simulation manually:

```bash
ros2 launch diffusion_policy_inference simulation_inference.launch.py \
  checkpoint_path:=/path/to/your/model_best.pth \
  model_dir:=/path/to/your/model/directory \
  inference_rate:=10.0 \
  log_level:=info
```

## Model Requirements

The inference node expects:

1. A PyTorch checkpoint file (`.pth`) containing:
   - `model_state_dict`: The model weights
   - `args`: Training arguments including model architecture parameters

2. Python modules in the specified `model_dir`:
   - `model.py`: Containing the `DiffusionPolicyModel` class
   - `train.py`: Containing diffusion sampling functions (`linear_beta_schedule`, `extract`, `p_sample`, `p_sample_loop`)

## Customization

You can customize the behavior of the inference node by modifying the parameters in the launch file or when running the node directly:

```bash
ros2 run diffusion_policy_inference model_inference_node \
  --ros-args \
  -p checkpoint_path:=/path/to/your/model_best.pth \
  -p model_dir:=/path/to/your/model/directory \
  -p inference_rate:=10.0 \
  -p image_topic:=/camera_head/color/image_raw \
  -p joint_states_topic:=/joint_states
```

## Troubleshooting

1. **Model loading errors**: Ensure your checkpoint file is compatible with the model architecture and contains all required components.

2. **Simulation issues**: If the simulation doesn't start properly, try cleaning up existing ROS processes:
   ```bash
   pkill -9 -f "ros2|gazebo|gz|rviz2|robot_state_publisher|move_group|cube_spawner|model_inference_node"
   ```

3. **Robot control issues**: Check that the joint names in the inference node match those in your simulation.

## License

MIT License

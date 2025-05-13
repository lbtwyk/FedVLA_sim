# Data Collection for Visuomotor Diffusion Policy

This document describes the data collection system for the cube stacking project, which is designed to gather trajectory data for training a visuomotor diffusion policy.

## Overview

The data collection system captures:
- Robot joint states
- Gripper positions
- Camera images
- Timestamps

All data is synchronized and saved in a structured format that can be used for training machine learning models.

## System Components

The data collection system consists of two main components:

1. **Trajectory Data Interfaces Package**
   - Custom ROS 2 interfaces for data collection
   - Service definitions for starting and stopping episode recording

2. **Trajectory Data Collector Package**
   - State Logger Node: Records joint states, gripper positions, and camera images
   - Provides services for starting and stopping data collection episodes

## Data Structure

Data is saved in the following directory structure:

```
~/mycobot_episodes/
└── episode_YYYYMMDD_HHMMSS_mmm/
    ├── states.json       # Joint states and gripper positions
    └── frame_dir/        # Camera images
        ├── image_00000.png
        ├── image_00001.png
        └── ...
```

The `states.json` file contains an array of entries, each with:
- `angles`: Array of 6 joint angles in radians
- `gripper_value`: Gripper position mapped to 0-100 range
- `image`: Path to the corresponding image file
- `timestamp`: Unix timestamp

## Usage

### Launching Data Collection

To launch the cube stacking simulation with data collection enabled:

```bash
ros2 launch mycobot_stacking_project collect_data.launch.py output_base_dir:=~/mycobot_episodes
```

Parameters:
- `output_base_dir`: Base directory for saving episode data (default: `~/mycobot_episodes`)
- `log_frequency_hz`: Frequency for logging data in Hz (default: 10.0)

### Collected Data

Each episode is automatically saved in a directory named with the timestamp when it started:
```
episode_YYYYMMDD_HHMMSS_mmm
```

For example: `episode_20250513_211104_730`

### Data Format

The `states.json` file contains an array of entries in the following format:

```json
[
  {
    "angles": [0.1, 0.2, 0.3, 0.4, 0.5, 0.6],
    "gripper_value": [50],
    "image": "frame_dir/image_00000.png"
  },
  ...
]
```

- `angles`: Joint angles in radians for the 6 robot joints
- `gripper_value`: Gripper position mapped from joint angle to 0-100 range (0=open, 100=closed)
- `image`: Relative path to the corresponding image file

## Implementation Details

### State Logger Node

The State Logger Node:
1. Subscribes to `/joint_states` and `/camera_head/color/image_raw` topics
2. Records data at the specified frequency (default: 10Hz)
3. Provides a service for starting and stopping episode recording
4. Saves data to the specified output directory

### Integration with Stacking Manager

The Stacking Manager Node:
1. Calls the State Logger service to start recording at the beginning of a stacking task
2. Calls the State Logger service to stop recording when the task is complete
3. Passes a unique episode identifier based on the current timestamp

## Troubleshooting

### Common Issues

- **Missing Data**: Ensure that the robot and camera are publishing data on the expected topics
- **Permission Errors**: Make sure the output directory is writable
- **Path Expansion**: The `~` character in paths is properly expanded to the home directory

### Checking Collected Data

To check if data was collected successfully:

```bash
ls -la ~/mycobot_episodes
```

To view the contents of a specific episode:

```bash
ls -la ~/mycobot_episodes/episode_YYYYMMDD_HHMMSS_mmm
```

To check the JSON data structure:

```bash
head -n 20 ~/mycobot_episodes/episode_YYYYMMDD_HHMMSS_mmm/states.json
```

## Using Collected Data

The collected data can be used to train a visuomotor diffusion policy. The data format is compatible with standard machine learning frameworks and can be processed using Python libraries such as:

- NumPy for numerical data
- OpenCV for image processing
- PyTorch or TensorFlow for model training

## License

This data collection system is licensed under the MIT License - see the LICENSE file for details.

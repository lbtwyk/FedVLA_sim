# Data Collection for Visuomotor Diffusion Policy

This document describes the data collection system for the cube stacking project, designed to gather trajectory data for training visuomotor diffusion policies.

## Overview

The data collection system captures:
- Robot joint states (6 arm joints)
- Gripper positions (mapped to 0-100 range)
- Camera images (424x240 resolution)
- Synchronized at 10Hz frequency

Data is saved in degrees format for better model sensitivity.

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
~/mycobot_episodes_degrees/
└── episode_YYYYMMDD_HHMMSS_mmm/
    ├── states.json       # Joint states and gripper positions
    └── frame_dir/        # Camera images
        ├── image_00000.png
        ├── image_00001.png
        └── ...
```

The `states.json` file contains an array of entries, each with:
- `angles`: Array of 6 joint angles in degrees
- `gripper_value`: Gripper position mapped to 0-100 range (0=open, 100=closed)
- `image`: Path to the corresponding image file

## Usage

### Launching Data Collection

To launch the cube stacking simulation with data collection enabled:

```bash
# Single episode (saves to ~/mycobot_episodes_degrees by default)
ros2 launch mycobot_stacking_project collect_data.launch.py

# Multiple episodes
./collect_multiple_episodes.sh
```

### Data Format

The `states.json` file contains an array of entries in the following format:

```json
[
  {
    "angles": [10.5, -15.2, 30.8, -45.1, 60.3, -75.6],
    "gripper_value": [50],
    "image": "frame_dir/image_00000.png"
  }
]
```

- `angles`: Joint angles in degrees for the 6 robot joints
- `gripper_value`: Gripper position mapped to 0-100 range (0=open, 100=closed)
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

- **Launch Errors**: Always run process cleanup before launching
- **Missing Data**: Ensure robot and camera are publishing on expected topics
- **Data Location**: Episodes save to `~/mycobot_episodes_degrees/` in degrees format

### Checking Collected Data

To check if data was collected successfully:

```bash
# Check episodes directory
ls -la ~/mycobot_episodes_degrees/

# Validate episodes
python3 check_episodes.py scan --dir ~/mycobot_episodes_degrees

# Check JSON data structure
head -n 10 ~/mycobot_episodes_degrees/episode_*/states.json
```

## Using Collected Data

The collected data is used to train visuomotor diffusion policy models:

- **Training**: Use the DP/ directory training system
- **Format**: Degrees format for better model sensitivity
- **Processing**: Compatible with PyTorch and standard ML frameworks

## License

This data collection system is licensed under the MIT License - see the LICENSE file for details.

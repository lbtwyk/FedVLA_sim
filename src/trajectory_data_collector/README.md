# Trajectory Data Collector

A ROS 2 package for collecting synchronized robot trajectory data including joint states, gripper positions, and camera images. This package is designed to gather training data for visuomotor diffusion policy models.

## Overview

The trajectory data collector provides a comprehensive system for recording robot demonstrations:

- **State Logger Node**: Records synchronized robot states and camera images
- **Episode Management**: Automatic episode creation and data organization
- **Flexible Configuration**: Configurable recording frequency and data formats
- **Service Interface**: Start/stop recording via ROS 2 services

## Package Structure

```
src/trajectory_data_collector/
├── package.xml                      # Package metadata
├── setup.py                         # Python package setup
├── setup.cfg                        # Package configuration
├── resource/                        # Package resources
├── trajectory_data_collector/
│   ├── __init__.py
│   └── state_logger_node.py         # Main data collection node
├── launch/
│   └── test_logger.launch.py        # Test launch file
└── test/                            # Unit tests
```

## Node: State Logger

**Executable**: `state_logger_node`

The main data collection node that records synchronized robot states and camera images.

### Subscribed Topics

- `/joint_states` (sensor_msgs/JointState): Robot joint positions and velocities
- `/camera_head/color/image_raw` (sensor_msgs/Image): Camera images from robot head

### Services

- `/start_stop_episode` (trajectory_data_interfaces/StartStopEpisode): Control episode recording

### Parameters

**Core Parameters**:
- `output_base_dir` (string): Base directory for saving episodes (default: "/tmp/mycobot_episodes")
- `log_frequency_hz` (double): Recording frequency in Hz (default: 10.0)
- `use_sim_time` (bool): Use simulation time (default: true)

**Robot Configuration**:
- `arm_joint_names` (string[]): Names of arm joints to record
- `gripper_joint_name` (string): Name of gripper joint
- `image_dir_name` (string): Directory name for images within episodes (default: "frame_dir")

**Gripper Mapping**:
- `gripper_value_min_rad` (double): Minimum gripper position in radians (default: -0.5)
- `gripper_value_max_rad` (double): Maximum gripper position in radians (default: 0.0)
- `gripper_value_output_min_int` (int): Minimum output value (default: 0)
- `gripper_value_output_max_int` (int): Maximum output value (default: 100)

### Data Format

Episodes are saved in the following structure:

```
output_base_dir/
└── episode_YYYYMMDD_HHMMSS_mmm/
    ├── states.json                  # Robot states and metadata
    └── frame_dir/                   # Camera images
        ├── image_00000.png
        ├── image_00001.png
        └── ...
```

**states.json Format**:
```json
[
  {
    "angles": [0.1, 0.2, 0.3, 0.4, 0.5, 0.6],
    "gripper_value": [50],
    "image": "frame_dir/image_00000.png",
    "timestamp": 1234567890.123
  },
  ...
]
```

- `angles`: Joint angles in degrees (converted from radians)
- `gripper_value`: Gripper position mapped to 0-100 range
- `image`: Relative path to corresponding image
- `timestamp`: Unix timestamp (optional)

## Usage

### Basic Launch

```bash
# Launch with default parameters
ros2 run trajectory_data_collector state_logger_node

# Launch with custom parameters
ros2 run trajectory_data_collector state_logger_node \
  --ros-args \
  -p output_base_dir:=~/my_episodes \
  -p log_frequency_hz:=5.0
```

### Using Launch File

```bash
# Test launch with default configuration
ros2 launch trajectory_data_collector test_logger.launch.py

# Custom configuration
ros2 launch trajectory_data_collector test_logger.launch.py \
  output_base_dir:=~/custom_episodes \
  log_frequency_hz:=20.0
```

### Service Interface

Control episode recording using the service interface:

```bash
# Start recording a new episode
ros2 service call /start_stop_episode trajectory_data_interfaces/srv/StartStopEpisode \
  "{command: 'start', episode_id: 'my_episode_001'}"

# Stop current recording
ros2 service call /start_stop_episode trajectory_data_interfaces/srv/StartStopEpisode \
  "{command: 'stop', episode_id: ''}"
```

### Integration with Stacking Manager

The state logger automatically integrates with the myCobot stacking manager:

```python
# In stacking manager node
self.episode_client = self.create_client(
    StartStopEpisode, '/start_stop_episode'
)

# Start episode
request = StartStopEpisode.Request()
request.command = 'start'
request.episode_id = f'episode_{timestamp}'
future = self.episode_client.call_async(request)

# Stop episode
request.command = 'stop'
future = self.episode_client.call_async(request)
```

## Configuration Examples

### High-Frequency Recording

For detailed motion capture:

```yaml
log_frequency_hz: 30.0
output_base_dir: "~/high_freq_episodes"
```

### Custom Joint Configuration

For different robot configurations:

```yaml
arm_joint_names: 
  - "joint1"
  - "joint2" 
  - "joint3"
  - "joint4"
  - "joint5"
  - "joint6"
gripper_joint_name: "gripper_joint"
```

### Gripper Value Mapping

Map gripper positions to custom ranges:

```yaml
gripper_value_min_rad: -1.0      # Fully closed
gripper_value_max_rad: 0.5       # Fully open
gripper_value_output_min_int: 0  # Output closed value
gripper_value_output_max_int: 255 # Output open value
```

## Data Processing

### Loading Episodes

```python
import json
import cv2
import os

def load_episode(episode_dir):
    # Load states
    with open(os.path.join(episode_dir, 'states.json'), 'r') as f:
        states = json.load(f)
    
    # Load images
    images = []
    for state in states:
        img_path = os.path.join(episode_dir, state['image'])
        image = cv2.imread(img_path)
        images.append(image)
    
    return states, images
```

### Data Validation

```python
def validate_episode(episode_dir):
    states_file = os.path.join(episode_dir, 'states.json')
    frame_dir = os.path.join(episode_dir, 'frame_dir')
    
    # Check files exist
    if not os.path.exists(states_file):
        return False, "Missing states.json"
    
    if not os.path.exists(frame_dir):
        return False, "Missing frame_dir"
    
    # Load and validate states
    with open(states_file, 'r') as f:
        states = json.load(f)
    
    if len(states) < 5:
        return False, "Too few frames"
    
    return True, "Valid episode"
```

## Troubleshooting

### Common Issues

1. **No data recorded**: Check topic names and ensure robot is publishing
2. **Missing images**: Verify camera topic and image encoding
3. **Permission errors**: Ensure output directory is writable
4. **Timing issues**: Adjust log frequency for system capabilities

### Debug Commands

```bash
# Check subscribed topics
ros2 topic list | grep -E 'joint_states|camera'

# Monitor data collection
ros2 topic echo /joint_states --once
ros2 topic echo /camera_head/color/image_raw --once

# Check service availability
ros2 service list | grep start_stop_episode

# View node parameters
ros2 param list /state_logger_node
```

### Log Analysis

```bash
# Check node logs
ros2 log view | grep state_logger

# Monitor episode creation
ls -la ~/mycobot_episodes/

# Validate episode data
python3 -c "
import json
with open('~/mycobot_episodes/episode_*/states.json') as f:
    data = json.load(f)
    print(f'Episode has {len(data)} frames')
"
```

## Dependencies

- ROS 2 Jazzy Jalisco
- sensor_msgs
- cv_bridge
- OpenCV Python
- trajectory_data_interfaces

## Related Packages

- **[trajectory_data_interfaces](../trajectory_data_interfaces/README.md)** - Service definitions
- **[mycobot_stacking_project](../mycobot_stacking_project/README.md)** - Main simulation package
- **[diffusion_policy_inference](../diffusion_policy_inference/README.md)** - Model inference

## Back to Main Documentation

← [Main Project README](../../README.md)

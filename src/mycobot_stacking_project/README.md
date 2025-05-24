# myCobot Stacking Project

The main ROS 2 package for cube stacking simulation using the myCobot 280 robot arm. This package provides traditional motion planning capabilities using MoveIt 2 and supports data collection for training diffusion policy models.

## Overview

This package implements a complete cube stacking pipeline with the following components:

- **Cube Spawner Node**: Manages cube placement and randomization in the simulation
- **Stacking Manager Node**: Executes pick-and-place operations using MoveIt 2
- **Gazebo Integration**: Custom world files and robot configurations
- **Data Collection**: Integration with trajectory data collection system

## Package Structure

```
src/mycobot_stacking_project/
├── CMakeLists.txt                   # Build configuration
├── package.xml                      # Package metadata
├── include/mycobot_stacking_project/
│   ├── cube_spawner_node.hpp        # Cube spawning functionality
│   └── stacking_manager_node.hpp    # Main stacking logic
├── src/
│   ├── cube_spawner_node.cpp        # Cube spawning implementation
│   └── stacking_manager_node.cpp    # Motion planning and control
├── launch/
│   ├── fixed_stacking_task.launch.py      # Traditional motion planning
│   ├── collect_data.launch.py             # Data collection mode
│   ├── mycobot_gazebo_with_friction.launch.py # Gazebo simulation
│   └── fixed_rviz.launch.py               # RViz configuration
├── worlds/
│   └── cube_stacking_world.world          # Gazebo world file
├── models/
│   ├── yellow_cube/                       # Yellow cube model
│   └── orange_cube/                       # Orange cube model
├── rviz/
│   └── stacking_display.rviz              # RViz configuration
├── urdf/
│   ├── mycobot_280_with_friction.urdf.xacro # Robot with friction
│   └── gripper_friction.urdf.xacro         # Gripper configuration
└── config/
    └── cube_positions.yaml                 # Cube position configurations
```

## Nodes

### Cube Spawner Node

**Executable**: `cube_spawner_node`

Manages cube placement in the simulation environment:

- Spawns yellow and orange cubes with randomized positions
- Adds cubes to the MoveIt planning scene
- Supports collision detection to prevent cube overlap
- Configurable position bounds and minimum distances

**Parameters**:
- `yellow_min_x`, `yellow_max_x`: X-axis bounds for yellow cube
- `yellow_min_y`, `yellow_max_y`: Y-axis bounds for yellow cube  
- `orange_min_x`, `orange_max_x`: X-axis bounds for orange cube
- `orange_min_y`, `orange_max_y`: Y-axis bounds for orange cube
- `min_distance_between_cubes`: Minimum distance between cubes
- `randomize_positions`: Enable/disable position randomization

### Stacking Manager Node

**Executable**: `stacking_manager_node`

Implements the cube stacking task using MoveIt 2:

- Waits for cubes to be detected in the planning scene
- Executes pick-and-place operations with robust error handling
- Supports both Cartesian and joint-space planning
- Integrates with data collection system when enabled

**Key Features**:
- Pre-placement positioning for precise cube placement
- Retry mechanisms for failed planning attempts
- Configurable gripper control and timing
- Speed control (75% of maximum for stability)

## Launch Files

### fixed_stacking_task.launch.py

Main launch file for traditional motion planning:

```bash
ros2 launch mycobot_stacking_project fixed_stacking_task.launch.py
```

**Parameters**:
- `use_sim_time`: Use simulation time (default: true)
- `log_level`: ROS log level (default: info)
- `robot_name`: Robot name (default: mycobot_280)

### collect_data.launch.py

Launch file for data collection mode:

```bash
ros2 launch mycobot_stacking_project collect_data.launch.py \
  output_base_dir:=~/mycobot_episodes \
  log_frequency_hz:=10.0
```

**Parameters**:
- `output_base_dir`: Directory for saving episode data
- `log_frequency_hz`: Data collection frequency
- Cube position parameters (same as cube spawner)

### mycobot_gazebo_with_friction.launch.py

Gazebo simulation with enhanced friction parameters:

```bash
ros2 launch mycobot_stacking_project mycobot_gazebo_with_friction.launch.py \
  world_file:=cube_stacking_world.world
```

## Configuration

### Cube Properties
- **Dimensions**: 2.5cm x 2.5cm x 2.5cm
- **Colors**: Yellow (target cube), Orange (base cube)
- **Physics**: Enhanced friction parameters to prevent slipping
- **Collision**: Automatic collision detection during spawning

### Robot Configuration
- **DOF**: 6-axis arm + 1 gripper joint
- **Planning**: OMPL algorithms with Cartesian path planning
- **Speed**: 75% of maximum velocity for stable operation
- **Gripper**: Position control from 0 (closed) to 100 (open)

### Timing Parameters
- **Gazebo Startup**: 10-second delay
- **Stacking Manager**: 45-second delay after Gazebo
- **Grasp Hold**: 1 second at grasp position
- **Release Confirmation**: 1 second after cube release

## Usage Examples

### Basic Stacking Task

```bash
# Clean up existing processes
pkill -9 -f "ros2|gazebo|gz|rviz2|robot_state_publisher|move_group"

# Launch stacking simulation
ros2 launch mycobot_stacking_project fixed_stacking_task.launch.py
```

### Data Collection

```bash
# Single episode with custom output directory
ros2 launch mycobot_stacking_project collect_data.launch.py \
  output_base_dir:=~/my_episodes \
  log_frequency_hz:=10.0

# With randomized cube positions
ros2 launch mycobot_stacking_project collect_data.launch.py \
  output_base_dir:=~/mycobot_episodes \
  yellow_min_x:=0.15 yellow_max_x:=0.25 \
  yellow_min_y:=-0.05 yellow_max_y:=0.05 \
  orange_min_x:=0.25 orange_max_x:=0.35 \
  orange_min_y:=-0.05 orange_max_y:=0.05 \
  min_distance_between_cubes:=0.08 \
  randomize_positions:=true
```

## Integration

### With Data Collection System

The stacking manager automatically integrates with the trajectory data collector when `data_collection_enabled` is set to true:

- Starts episode recording before task execution
- Stops recording after task completion
- Provides episode metadata and status

### With Diffusion Policy Inference

The package provides the simulation environment for diffusion policy inference:

- Same Gazebo world and robot configuration
- Compatible cube spawning and positioning
- Consistent camera setup and timing

## Troubleshooting

### Common Issues

1. **Cubes not spawning**: Check Gazebo model paths and ensure models are properly installed
2. **Planning failures**: Increase planning time or adjust cube positions
3. **Gripper issues**: Verify gripper controller configuration and joint limits
4. **Timing issues**: Ensure proper delays between node startups

### Debug Commands

```bash
# Check planning scene
ros2 topic echo /planning_scene

# Monitor cube spawner status
ros2 topic echo /cube_spawner/status

# Check MoveIt planning
ros2 topic echo /move_group/status

# View robot state
ros2 topic echo /joint_states
```

## Dependencies

- ROS 2 Jazzy Jalisco
- MoveIt 2
- Gazebo Garden
- mycobot_ros2 packages
- trajectory_data_interfaces (for data collection)

## Back to Main Documentation

← [Main Project README](../../README.md)

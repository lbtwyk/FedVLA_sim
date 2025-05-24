# myCobot Stacking Project

The main ROS 2 package for cube stacking simulation using the myCobot 280 robot arm. This package provides traditional motion planning using MoveIt 2 and supports data collection for training diffusion policy models.

## Overview

This package implements a complete cube stacking pipeline:

- **Cube Spawner Node**: Manages cube placement in the simulation
- **Stacking Manager Node**: Executes pick-and-place operations using MoveIt 2
- **Gazebo Integration**: Custom world files and robot configurations with enhanced friction
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

- Spawns yellow and orange cubes at fixed or randomized positions
- Adds cubes to the MoveIt planning scene as collision objects
- Adds ground plane collision object
- Configurable position bounds and minimum distances

### Stacking Manager Node

**Executable**: `stacking_manager_node`

Implements the cube stacking task using MoveIt 2:

- Executes pick-and-place operations with robust error handling
- Supports both Cartesian and joint-space planning with retry mechanisms
- Integrates with data collection system when enabled
- Direct gripper control (not using MoveIt gripper control)

**Key Features**:
- Movement heights: Pre-grasp 0.15m, Grasp 0.11m, Lift 0.15m, Place 0.14m
- Gripper range: -0.5 (closed) to 0.0 (open)
- Speed control: 75% of maximum velocity
- Timing: 0.2s confirmation times with immediate retry on failure

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
- **Positions**: Yellow at (0, 0.20), Orange at (0.035, 0.25)
- **Heights**: World cubes at z=0.05m, planning scene at z=0.0125m
- **Ground plane**: Planning scene at z=-0.005m
- **Physics**: Enhanced friction parameters to prevent slipping

### Robot Configuration
- **DOF**: 6-axis arm + 1 gripper joint
- **Planning**: OMPL algorithms with Cartesian path planning and retry mechanisms
- **Speed**: 75% of maximum velocity for stable operation
- **Gripper**: Direct control from -0.5 (closed) to 0.0 (open)

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
# Single episode (saves to ~/mycobot_episodes_degrees by default)
ros2 launch mycobot_stacking_project collect_data.launch.py

# With custom output directory
ros2 launch mycobot_stacking_project collect_data.launch.py \
  output_base_dir:=~/my_episodes_degrees
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

1. **Launch failures**: Always run process cleanup before launching
2. **Planning failures**: Rebuild workspace before launching
3. **Gripper issues**: Gripper uses direct control, not MoveIt gripper control
4. **Data collection**: Episodes save to `~/mycobot_episodes_degrees/` in degrees format

### Debug Commands

```bash
# Check planning scene
ros2 topic echo /planning_scene

# Check MoveIt planning
ros2 topic echo /move_group/status

# View robot state
ros2 topic echo /joint_states

# Process cleanup
pkill -9 -f "ros2|gazebo|gz|rviz2|robot_state_publisher|move_group|cube_spawner|stacking_manager"
```

## Dependencies

- ROS 2 Jazzy Jalisco
- MoveIt 2
- Gazebo Garden
- mycobot_ros2 packages
- trajectory_data_interfaces (for data collection)

## Back to Main Documentation

← [Main Project README](../../README.md)

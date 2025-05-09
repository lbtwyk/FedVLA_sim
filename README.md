# FedVLA_sim - Cube Stacking Project

A ROS 2 simulation project for robotic cube stacking using the myCobot 280 robot arm in Gazebo. This project demonstrates precise manipulation capabilities by having the robot pick up a yellow cube and place it on top of an orange cube.

## Overview

This project implements a complete cube stacking pipeline in a simulated environment:

1. Spawns colored cubes (yellow and orange) in a Gazebo simulation
2. Adds the cubes to the MoveIt planning scene
3. Controls the myCobot 280 robot arm to:
   - Locate the yellow cube
   - Pick it up with the gripper
   - Move it to the orange cube
   - Place it precisely on top
   - Return to the home position

The project uses ROS 2 Jazzy Jalisco and MoveIt 2 for motion planning and control.

## System Requirements

- Ubuntu 24.04 (can run in WSL2 with GPU acceleration)
- ROS 2 Jazzy Jalisco
- Gazebo Garden
- MoveIt 2
- NVIDIA GPU with WSL2 drivers (for optimal performance)

## Project Structure

```
mycobot_stacking_project/
├── include/                  # Header files
│   └── mycobot_stacking_project/
│       ├── cube_spawner_node.hpp     # Cube spawning functionality
│       └── stacking_manager_node.hpp # Main stacking logic
├── src/                      # Source files
│   ├── cube_spawner_node.cpp         # Spawns cubes in the planning scene
│   └── stacking_manager_node.cpp     # Implements the stacking task
├── launch/                   # Launch files
│   ├── fixed_stacking_task.launch.py # Main launch file for the project
│   ├── mycobot_gazebo_with_friction.launch.py # Gazebo with friction parameters
│   └── fixed_rviz.launch.py          # RViz configuration
├── worlds/                   # Gazebo world files
│   └── cube_stacking_world.world     # World with cubes and ground plane
├── models/                   # Gazebo models
│   ├── yellow_cube/                  # Yellow cube model
│   └── orange_cube/                  # Orange cube model
├── rviz/                     # RViz configuration
│   └── stacking_display.rviz         # RViz config for the project
└── urdf/                     # Robot URDF files
    ├── mycobot_280_with_friction.urdf.xacro # Robot with friction parameters
    └── gripper_friction.urdf.xacro          # Gripper with friction parameters
```

## Key Components


### Stacking Manager Node

The `StackingManagerNode` implements the core stacking logic:

- Waits for the cubes to be detected in the planning scene
- Moves the robot arm to the home position
- Opens the gripper
- Approaches the yellow cube
- Closes the gripper to grasp the cube
- Lifts the cube
- Moves to the orange cube
- Places the yellow cube on top of the orange cube
- Opens the gripper to release the cube
- Returns to the home position

## Installation

1. Create a ROS 2 workspace:
   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws/src
   ```

2. Clone the repository:
   ```bash
   git clone https://github.com/lbtwyk/FedVLA_sim.git .
   ```

3. Install dependencies:
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

## Usage

1. Make sure to terminate any existing ROS 2 or Gazebo processes:
   ```bash
   pkill -9 -f "ros2|gazebo|gz|rviz2|robot_state_publisher|move_group|cube_spawner|stacking_manager"
   ```

2. Launch the cube stacking simulation:
   ```bash
   ros2 launch mycobot_stacking_project fixed_stacking_task.launch.py
   ```

3. The simulation will:
   - Start Gazebo with the cube stacking world
   - Launch MoveIt and RViz
   - Spawn the cubes in the planning scene
   - Execute the stacking task automatically

## Key Features

- **Enhanced Friction**: Increased friction parameters to prevent cubes from slipping during manipulation
- **Robust Motion Planning**: Uses MoveIt's planning capabilities with increased planning time and attempts for challenging moves
- **Error Handling**: Comprehensive error detection and recovery mechanisms
- **Gripper Control**: Precise gripper control for reliable grasping and releasing

## Troubleshooting

- **Cube Slipping**: If cubes slip during grasping, check the friction parameters in the URDF and world files
- **Planning Failures**: Increase planning time and attempts in the stacking manager node
- **Gripper Issues**: Verify gripper joint limits and controller configuration
- **Launch Errors**: Ensure all previous simulation processes are terminated before launching

## Development

- Use RViz's motion planning panel for interactive parameter adjustment
- Rebuild the code every time before launching to avoid timing errors
- The cube dimensions are set to 0.025 units (2.5cm)

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Author

Yukun Wang (lambert.wang.23@ucl.ac.uk)

#!/usr/bin/env python3
"""
Launch file for running diffusion policy inference in simulation.

This launch file:
1. Starts Gazebo with the cube stacking world
2. Launches MoveIt for the myCobot 280 robot
3. Spawns cubes in the simulation
4. Starts the model inference node
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription, ExecuteProcess, TimerAction,
    DeclareLaunchArgument, AppendEnvironmentVariable, GroupAction
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.conditions import IfCondition


def generate_launch_description():
    """Generate a launch description for simulation inference."""
    # Declare launch arguments
    declared_arguments = [
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation time"
        ),
        DeclareLaunchArgument(
            "log_level",
            default_value="info",
            description="Log level"
        ),
        DeclareLaunchArgument(
            "robot_name",
            default_value="mycobot_280",
            description="Robot name"
        ),
        DeclareLaunchArgument(
            "checkpoint_path",
            default_value="",
            description="Path to the model checkpoint file"
        ),
        DeclareLaunchArgument(
            "model_dir",
            default_value="",
            description="Directory containing model.py and train.py"
        ),
        DeclareLaunchArgument(
            "inference_rate",
            default_value="10.0",
            description="Rate at which to run inference (Hz)"
        )
    ]

    # Variables
    use_sim_time = LaunchConfiguration("use_sim_time")
    log_level = LaunchConfiguration("log_level")
    robot_name = LaunchConfiguration("robot_name")
    checkpoint_path = LaunchConfiguration("checkpoint_path")
    model_dir = LaunchConfiguration("model_dir")
    inference_rate = LaunchConfiguration("inference_rate")

    # Package paths
    pkg_mycobot_stacking_project = get_package_share_directory('mycobot_stacking_project')
    pkg_mycobot_gazebo = get_package_share_directory('mycobot_gazebo')
    pkg_mycobot_moveit_config = get_package_share_directory('mycobot_moveit_config')

    # Find world path
    world_path = os.path.join(
        pkg_mycobot_stacking_project,
        'worlds', 'cube_stacking_world.world'
    )

    # Setup Gazebo model resource path for custom cube models
    models_path = os.path.join(pkg_mycobot_stacking_project, 'models')
    set_models_env = AppendEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=models_path
    )

    # --- Include other launch files ---

    # Gazebo launch with custom friction parameters
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_mycobot_stacking_project, 'launch', 'mycobot_gazebo_with_friction.launch.py')
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'world_file': world_path,
            'z': '0.001'
        }.items()
    )

    # MoveIt launch
    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_mycobot_moveit_config, 'launch', 'move_group.launch.py')
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )

    # Planning scene is handled by move_group

    # RViz configuration
    rviz_config_path = os.path.join(
        pkg_mycobot_stacking_project, 'rviz', 'stacking_display.rviz'
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Cube Spawner Node
    cube_spawner_node = Node(
        package='mycobot_stacking_project',
        executable='cube_spawner_node',
        name='cube_spawner',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Add a delay before starting the cube spawner
    delayed_cube_spawner = TimerAction(
        period=5.0,  # Start after 5 seconds
        actions=[cube_spawner_node]
    )

    # Model Inference Node
    model_inference_node = Node(
        package='diffusion_policy_inference',
        executable='model_inference_node',
        name='model_inference_node',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'checkpoint_path': checkpoint_path},
            {'model_dir': model_dir},
            {'inference_rate': inference_rate},
            {'image_topic': '/camera_head/color/image_raw'},
            {'joint_states_topic': '/joint_states'},
            {'arm_joint_names': [
                'link1_to_link2', 'link2_to_link3', 'link3_to_link4',
                'link4_to_link5', 'link5_to_link6', 'link6_to_link6_flange'
            ]},
            {'gripper_joint_name': 'gripper_controller'}
        ]
    )

    # Add a delay before starting the model inference node
    delayed_model_inference = TimerAction(
        period=10.0,  # Start after 10 seconds
        actions=[model_inference_node]
    )

    # Set default camera position in Gazebo
    set_camera_position = ExecuteProcess(
        cmd=['gz', 'service', '-s', '/gui/move_to/pose',
             '--reqtype', 'gz.msgs.GUICamera',
             '--reptype', 'gz.msgs.Boolean',
             '--timeout', '2000',
             '--req', 'pose: {position: {x: 0.56, y: 0.46, z: 0.36} orientation: {x: 0.02, y: 0.01, z: -0.93, w: 0.36}}'],
        output='screen'
    )

    # Add a delay before setting camera position
    delayed_camera_position = TimerAction(
        period=5.0,  # Start after 5 seconds
        actions=[set_camera_position]
    )

    return LaunchDescription(
        declared_arguments + [
            set_models_env,
            gazebo_launch,
            move_group_launch,
            rviz_node,
            delayed_cube_spawner,
            delayed_camera_position,
            delayed_model_inference
        ]
    )

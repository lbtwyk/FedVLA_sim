import os
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    TimerAction,
    AppendEnvironmentVariable,
    RegisterEventHandler,
    ExecuteProcess
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # Declare launch arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation (Gazebo) clock if true",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "log_level",
            default_value="info",
            description="Log level for nodes",
        )
    )

    # Variables
    use_sim_time = LaunchConfiguration("use_sim_time")
    log_level = LaunchConfiguration("log_level")
    pkg_mycobot_stacking_project = FindPackageShare('mycobot_stacking_project')
    pkg_mycobot_gazebo = FindPackageShare('mycobot_gazebo')
    pkg_mycobot_moveit_config = FindPackageShare('mycobot_moveit_config')

    # Setup Gazebo model resource path for custom cube models
    models_path = PathJoinSubstitution([pkg_mycobot_stacking_project, 'models'])
    set_models_env = AppendEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=models_path
    )

    # Find paths
    world_path = os.path.join(
        FindPackageShare('mycobot_stacking_project').find('mycobot_stacking_project'),
        'worlds', 'cube_stacking_world.world'
    )
    rviz_config_path = PathJoinSubstitution([
        pkg_mycobot_stacking_project, 'rviz', 'stacking_display.rviz'
    ])

    # --- Include other launch files ---

    # Gazebo launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_mycobot_gazebo, 'launch', 'mycobot.gazebo.launch.py'])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'load_controllers': 'true', # Load controllers for interaction
            'use_camera': 'true',       # Enable the camera
            'world_file': world_path,        # Correct argument name: world_file
            'use_rviz': 'false'         # Explicitly disable RViz from Gazebo launch
        }.items()
    )

    # MoveGroup launch
    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_mycobot_moveit_config, 'launch', 'move_group.launch.py'])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'launch_rviz': 'false', # Explicitly disable RViz from MoveIt launch
            # Add other necessary MoveIt args if needed
        }.items()
    )

    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': use_sim_time}],
        # respawn=True, # Optional: restart RViz if it crashes
    )

    # --- Nodes for this project ---

    # Find cube detector parameters
    cube_detector_params_path = PathJoinSubstitution([
        pkg_mycobot_stacking_project, 'config', 'cube_detector_params.yaml'
    ])

    # Perception Node
    cube_detector_node = Node(
        package='mycobot_stacking_project',
        executable='cube_detector_node', # Use the C++ executable name
        name='cube_detector',
        output='screen',
        arguments=["--ros-args", "--log-level", log_level],
        parameters=[
            {'use_sim_time': use_sim_time},
            cube_detector_params_path
        ]
    )

    # Application Node (Stacking Manager)
    stacking_manager_node = Node(
        package='mycobot_stacking_project',
        executable='stacking_manager_node', # Use the C++ executable name
        name='stacking_manager',
        output='screen',
        arguments=["--ros-args", "--log-level", log_level],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Load controllers in sequence
    # First, load the joint state broadcaster
    load_joint_state_broadcaster_cmd = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    # Then, load the arm controller after joint state broadcaster is loaded
    load_arm_controller_cmd = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'arm_controller'],
        output='screen'
    )

    # Finally, load the gripper controller after arm controller is loaded
    load_gripper_controller_cmd = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'gripper_action_controller'],
        output='screen'
    )

    # Register event handlers for sequencing
    # Launch the arm controller after joint state broadcaster is loaded
    arm_controller_event_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_joint_state_broadcaster_cmd,
            on_exit=[load_arm_controller_cmd]
        )
    )

    # Launch the gripper controller after arm controller is loaded
    gripper_controller_event_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_arm_controller_cmd,
            on_exit=[load_gripper_controller_cmd]
        )
    )

    # Launch the stacking manager after gripper controller is loaded
    stacking_manager_event_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_gripper_controller_cmd,
            on_exit=[stacking_manager_node]
        )
    )

    # Add a delay before starting the controller loading sequence
    # to ensure Gazebo and MoveIt are fully initialized
    delayed_controller_loading = TimerAction(
        period=15.0,  # Start after 15 seconds
        actions=[load_joint_state_broadcaster_cmd]
    )

    # Cube Spawner Node
    cube_spawner_node = Node(
        package='mycobot_stacking_project',
        executable='cube_spawner_node',
        name='cube_spawner',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription(
        declared_arguments + [
        set_models_env,
        gazebo_launch,
        move_group_launch,
        rviz_node,
        cube_detector_node,
        cube_spawner_node,
        # Add the controller loading sequence with event handlers
        delayed_controller_loading,
        arm_controller_event_handler,
        gripper_controller_event_handler,
        stacking_manager_event_handler,
    ])
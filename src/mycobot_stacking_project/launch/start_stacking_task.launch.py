import os
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    TimerAction,
    AppendEnvironmentVariable
)
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

    # Add a delay before starting the stacking manager
    # to ensure Gazebo and MoveIt are fully initialized
    delayed_stacking_manager = TimerAction(
        period=30.0,  # Start after 30 seconds
        actions=[stacking_manager_node]
    )

    # Cube Spawner Node
    cube_spawner_node = Node(
        package='mycobot_stacking_project',
        executable='cube_spawner_node',
        name='cube_spawner',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Load controllers with a delay
    load_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
    )

    load_arm_controller = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['arm_controller', '--controller-manager', '/controller_manager'],
    )

    load_gripper_controller = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['gripper_action_controller', '--controller-manager', '/controller_manager'],
    )

    # Add delays for loading controllers
    delayed_joint_state_broadcaster = TimerAction(
        period=10.0,  # Start after 10 seconds
        actions=[load_joint_state_broadcaster]
    )

    delayed_arm_controller = TimerAction(
        period=15.0,  # Start after 15 seconds
        actions=[load_arm_controller]
    )

    delayed_gripper_controller = TimerAction(
        period=20.0,  # Start after 20 seconds
        actions=[load_gripper_controller]
    )

    return LaunchDescription(
        declared_arguments + [
        set_models_env,
        gazebo_launch,
        move_group_launch,
        rviz_node,
        cube_detector_node,
        cube_spawner_node,
        # Add controller loading with delays
        delayed_joint_state_broadcaster,
        delayed_arm_controller,
        delayed_gripper_controller,
        # Add the stacking manager with a delay to ensure Gazebo and MoveIt are initialized
        delayed_stacking_manager,
    ])
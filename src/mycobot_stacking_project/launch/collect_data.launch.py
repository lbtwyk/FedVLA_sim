import os
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    TimerAction,
    AppendEnvironmentVariable,
    OpaqueFunction,
    ExecuteProcess,
    GroupAction,
    RegisterEventHandler
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder

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
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_name",
            default_value="mycobot_280",
            description="Name of the robot to use",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "output_base_dir",
            default_value="/tmp/mycobot_episodes",
            description="Base directory for saving episode data",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "log_frequency_hz",
            default_value="10.0",
            description="Frequency for logging data in Hz",
        )
    )
    # Cube randomization parameters (±0.005 from cube_stacking_world.world positions)
    # Yellow cube parameters (world position: 0, 0.20)
    declared_arguments.append(
        DeclareLaunchArgument(
            "yellow_min_x",
            default_value="-0.005",
            description="Minimum X coordinate for yellow cube placement",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "yellow_max_x",
            default_value="0.005",
            description="Maximum X coordinate for yellow cube placement",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "yellow_min_y",
            default_value="0.195",
            description="Minimum Y coordinate for yellow cube placement",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "yellow_max_y",
            default_value="0.205",
            description="Maximum Y coordinate for yellow cube placement",
        )
    )

    # Orange cube parameters (world position: 0.035, 0.25)
    declared_arguments.append(
        DeclareLaunchArgument(
            "orange_min_x",
            default_value="0.030",
            description="Minimum X coordinate for orange cube placement",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "orange_max_x",
            default_value="0.040",
            description="Maximum X coordinate for orange cube placement",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "orange_min_y",
            default_value="0.245",
            description="Minimum Y coordinate for orange cube placement",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "orange_max_y",
            default_value="0.255",
            description="Maximum Y coordinate for orange cube placement",
        )
    )

    # Common parameters
    declared_arguments.append(
        DeclareLaunchArgument(
            "min_distance_between_cubes",
            default_value="0.03",
            description="Minimum distance between cube centers",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "randomize_positions",
            default_value="true",
            description="Whether to randomize cube positions",
        )
    )

    # Variables
    use_sim_time = LaunchConfiguration("use_sim_time")
    log_level = LaunchConfiguration("log_level")
    robot_name = LaunchConfiguration("robot_name")
    output_base_dir = LaunchConfiguration("output_base_dir")
    log_frequency_hz = LaunchConfiguration("log_frequency_hz")

    # Cube randomization variables
    # Yellow cube parameters
    yellow_min_x = LaunchConfiguration("yellow_min_x")
    yellow_max_x = LaunchConfiguration("yellow_max_x")
    yellow_min_y = LaunchConfiguration("yellow_min_y")
    yellow_max_y = LaunchConfiguration("yellow_max_y")

    # Orange cube parameters
    orange_min_x = LaunchConfiguration("orange_min_x")
    orange_max_x = LaunchConfiguration("orange_max_x")
    orange_min_y = LaunchConfiguration("orange_min_y")
    orange_max_y = LaunchConfiguration("orange_max_y")

    # Common parameters
    min_distance_between_cubes = LaunchConfiguration("min_distance_between_cubes")
    randomize_positions = LaunchConfiguration("randomize_positions")

    pkg_mycobot_stacking_project = FindPackageShare('mycobot_stacking_project')
    pkg_mycobot_moveit_config = FindPackageShare('mycobot_moveit_config')

    # Setup Gazebo model resource path for custom cube models
    models_path = PathJoinSubstitution([pkg_mycobot_stacking_project, 'models'])
    set_models_env = AppendEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=models_path
    )

    # Find paths
    template_world_path = os.path.join(
        FindPackageShare('mycobot_stacking_project').find('mycobot_stacking_project'),
        'worlds', 'cube_stacking_world.world'
    )

    # Create a temporary directory for the generated world file
    temp_dir = '/tmp/mycobot_worlds'
    os.makedirs(temp_dir, exist_ok=True)

    # Path for the generated world file
    world_path = os.path.join(temp_dir, 'cube_stacking_world_randomized.world')
    rviz_config_path = PathJoinSubstitution([
        pkg_mycobot_stacking_project, 'rviz', 'stacking_display.rviz'
    ])

    # --- Include other launch files ---

    # Gazebo launch with custom friction parameters
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_mycobot_stacking_project, 'launch', 'mycobot_gazebo_with_friction.launch.py'])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'world_file': world_path,
            'z': '0.001'
        }.items()
    )

    # MoveGroup launch
    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_mycobot_moveit_config, 'launch', 'move_group.launch.py'])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'launch_rviz': 'false',
            'robot_name': robot_name,
        }.items()
    )

    # Note: Planning scene functionality is provided by move_group
    # No separate planning scene server needed

    # Create a function to configure MoveIt and RViz
    def configure_moveit_rviz(context):
        # Get the robot name as a string
        robot_name_str = robot_name.perform(context)

        # Get package path
        pkg_share_moveit_config_path = pkg_mycobot_moveit_config.find('mycobot_moveit_config')

        # Construct file paths using robot name string
        config_path = os.path.join(pkg_share_moveit_config_path, 'config', robot_name_str)

        # Define all config file paths
        joint_limits_file_path = os.path.join(config_path, 'joint_limits.yaml')
        kinematics_file_path = os.path.join(config_path, 'kinematics.yaml')
        moveit_controllers_file_path = os.path.join(config_path, 'moveit_controllers.yaml')
        srdf_model_path = os.path.join(config_path, f'{robot_name_str}.srdf')
        pilz_cartesian_limits_file_path = os.path.join(config_path, 'pilz_cartesian_limits.yaml')

        # Create MoveIt configuration
        moveit_config = (
            MoveItConfigsBuilder(robot_name_str, package_name='mycobot_moveit_config')
            .trajectory_execution(file_path=moveit_controllers_file_path)
            .robot_description_semantic(file_path=srdf_model_path)
            .joint_limits(file_path=joint_limits_file_path)
            .robot_description_kinematics(file_path=kinematics_file_path)
            .planning_pipelines(
                pipelines=["ompl", "pilz_industrial_motion_planner", "stomp"],
                default_planning_pipeline="ompl"
            )
            .planning_scene_monitor(
                publish_robot_description=True,
                publish_robot_description_semantic=True,
                publish_planning_scene=True,
            )
            .pilz_cartesian_limits(file_path=pilz_cartesian_limits_file_path)
            .to_moveit_configs()
        )

        # Create RViz node with proper MoveIt configuration
        rviz_node_configured = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path],
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.planning_pipelines,
                moveit_config.robot_description_kinematics,
                moveit_config.joint_limits,
                {'use_sim_time': use_sim_time}
            ],
        )

        return [rviz_node_configured]

    # Use OpaqueFunction to configure RViz with MoveIt parameters
    rviz_node = OpaqueFunction(function=configure_moveit_rviz)

    # --- Nodes for this project ---

    # Cube Spawner Node - Using direct cube spawning instead of perception
    cube_spawner_node = Node(
        package='mycobot_stacking_project',
        executable='cube_spawner_node',
        name='cube_spawner',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'yellow_min_x': yellow_min_x},
            {'yellow_max_x': yellow_max_x},
            {'yellow_min_y': yellow_min_y},
            {'yellow_max_y': yellow_max_y},
            {'orange_min_x': orange_min_x},
            {'orange_max_x': orange_max_x},
            {'orange_min_y': orange_min_y},
            {'orange_max_y': orange_max_y},
            {'min_distance_between_cubes': min_distance_between_cubes},
            {'randomize_positions': randomize_positions},
            {'template_world_path': template_world_path},
            {'output_world_path': world_path}
        ]
    )

    # Image Bridge for camera data
    # This bridges Gazebo camera topics to ROS topics for data collection
    image_bridge_node = Node(
        package='ros_gz_image',
        executable='image_bridge',
        name='image_bridge',
        output='screen',
        arguments=[
            '/camera_head/depth_image',
            '/camera_head/image',
        ],
        remappings=[
            ('/camera_head/depth_image', '/camera_head/depth/image_rect_raw'),
            ('/camera_head/image', '/camera_head/color/image_raw'),
        ],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # State Logger Node for data collection
    state_logger_node = Node(
        package='trajectory_data_collector',
        executable='state_logger_node',
        name='state_logger_node',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'output_base_dir': output_base_dir},
            {'log_frequency_hz': log_frequency_hz},
            {'arm_joint_names': ['link1_to_link2', 'link2_to_link3', 'link3_to_link4', 'link4_to_link5', 'link5_to_link6', 'link6_to_link6_flange']},
            {'gripper_joint_name': 'gripper_controller'},
            {'image_dir_name': 'frame_dir'},
            {'gripper_value_min_rad': -0.5},  # Closed position
            {'gripper_value_max_rad': 0.0},   # Open position
            {'gripper_value_output_min_int': 0},
            {'gripper_value_output_max_int': 100}
        ]
    )

    # Application Node (Stacking Manager)
    stacking_manager_node = Node(
        package='mycobot_stacking_project',
        executable='stacking_manager_node',
        name='stacking_manager',
        output='screen',
        arguments=["--ros-args", "--log-level", log_level],
        parameters=[
            {'use_sim_time': use_sim_time},
            {'data_collection_enabled': True}
        ]
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

    # Create parallel initialization groups with appropriate delays
    # Group 1: Set camera position after Gazebo is initialized
    parallel_group_1 = GroupAction([
        # Set camera position after Gazebo is initialized
        TimerAction(
            period=5.0,  # Reduced from 8.0 seconds
            actions=[set_camera_position]
        )
    ])

    # Group 2: Start image bridge, state logger and stacking manager in parallel
    # All depend on Gazebo, MoveIt, and cube spawner being initialized
    parallel_group_2 = GroupAction([
        # Start image bridge early to provide camera data
        TimerAction(
            period=8.0,  # Start image bridge early
            actions=[image_bridge_node]
        ),
        # Start state logger after image bridge is ready
        TimerAction(
            period=10.0,  # Increased from 8.0 seconds to give more time for controllers to initialize
            actions=[state_logger_node]
        ),
        # Start stacking manager slightly later to ensure state logger is ready
        TimerAction(
            period=15.0,  # Increased from 10.0 seconds to give more time for controllers to initialize
            actions=[stacking_manager_node]
        )
    ])

    # Register an event handler to launch Gazebo only after the cube spawner has exited
    # This ensures the world file is fully generated before Gazebo tries to load it
    gazebo_launch_after_cube_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=cube_spawner_node,
            on_exit=[gazebo_launch]
        )
    )

    return LaunchDescription(
        declared_arguments + [
        set_models_env,
        # Run cube spawner first to generate the world file with randomized cube positions
        cube_spawner_node,
        # Then start Gazebo with the generated world file (only after cube spawner has exited)
        gazebo_launch_after_cube_spawner,
        move_group_launch,
        rviz_node,
        # Add parallel group 1: camera position
        parallel_group_1,
        # Add parallel group 2: state logger and stacking manager
        parallel_group_2
    ])

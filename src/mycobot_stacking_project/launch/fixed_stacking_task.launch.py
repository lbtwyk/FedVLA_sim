import os
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    TimerAction,
    AppendEnvironmentVariable,
    OpaqueFunction,
    ExecuteProcess
)
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

    # Variables
    use_sim_time = LaunchConfiguration("use_sim_time")
    log_level = LaunchConfiguration("log_level")
    robot_name = LaunchConfiguration("robot_name")
    pkg_mycobot_stacking_project = FindPackageShare('mycobot_stacking_project')
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

    # Gazebo launch with custom friction parameters
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_mycobot_stacking_project, 'launch', 'mycobot_gazebo_with_friction.launch.py'])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'world_file': world_path,   # Correct argument name: world_file
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
            'launch_rviz': 'false', # Explicitly disable RViz from MoveIt launch
            'robot_name': robot_name, # Pass the robot name parameter
            # Add other necessary MoveIt args if needed
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
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Add a delay before starting the cube spawner
    # to ensure MoveIt is fully initialized
    delayed_cube_spawner = TimerAction(
        period=5.0,  # Start after 5 seconds (reduced from 15)
        actions=[cube_spawner_node]
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
        period=15.0,  # Start after 15 seconds (reduced from 45)
        actions=[stacking_manager_node]
    )

    # Set default camera position in Gazebo (position values rounded to 2 decimal places)
    set_camera_position = ExecuteProcess(
        cmd=['gz', 'service', '-s', '/gui/move_to/pose',
             '--reqtype', 'gz.msgs.GUICamera',
             '--reptype', 'gz.msgs.Boolean',
             '--timeout', '2000',
             '--req', 'pose: {position: {x: 0.56, y: 0.46, z: 0.36} orientation: {x: 0.02, y: 0.01, z: -0.93, w: 0.36}}'],
        output='screen'
    )

    # Add a delay before setting the camera position to ensure Gazebo is fully initialized
    delayed_camera_position = TimerAction(
        period=8.0,  # Start after 8 seconds
        actions=[set_camera_position]
    )

    return LaunchDescription(
        declared_arguments + [
        set_models_env,
        gazebo_launch,
        # Add the camera position setting with a delay
        delayed_camera_position,
        move_group_launch,
        rviz_node,
        # Add the cube spawner with a delay
        delayed_cube_spawner,
        # Add the stacking manager with a delay to ensure Gazebo and MoveIt are initialized
        delayed_stacking_manager,
    ])

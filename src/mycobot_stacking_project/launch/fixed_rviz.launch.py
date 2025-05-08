import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
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
            "robot_name",
            default_value="mycobot_280",
            description="Name of the robot to use",
        )
    )

    # Variables
    use_sim_time = LaunchConfiguration("use_sim_time")
    robot_name = LaunchConfiguration("robot_name")
    pkg_mycobot_stacking_project = FindPackageShare('mycobot_stacking_project')
    pkg_mycobot_moveit_config = FindPackageShare('mycobot_moveit_config')

    # Find paths
    rviz_config_path = PathJoinSubstitution([
        pkg_mycobot_stacking_project, 'rviz', 'stacking_display.rviz'
    ])

    # Get the robot name as a string
    robot_name_str = "mycobot_280"  # Hardcoded for simplicity

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
    rviz_node = Node(
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

    return LaunchDescription(
        declared_arguments + [
        rviz_node,
    ])

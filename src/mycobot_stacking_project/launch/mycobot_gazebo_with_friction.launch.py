import os
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    RegisterEventHandler,
    ExecuteProcess,
    SetEnvironmentVariable
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

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
            "world_file",
            default_value="cube_stacking_world.world",
            description="World file to load",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "z",
            default_value="0.0",
            description="Z position of the robot",
        )
    )

    # Variables
    use_sim_time = LaunchConfiguration("use_sim_time")
    world_file = LaunchConfiguration("world_file")
    z_position = LaunchConfiguration("z")
    
    # Package paths
    pkg_mycobot_gazebo = FindPackageShare('mycobot_gazebo')
    pkg_mycobot_stacking_project = FindPackageShare('mycobot_stacking_project')
    
    # Find the custom URDF file with friction parameters
    custom_urdf_path = os.path.join(
        get_package_share_directory('mycobot_stacking_project'),
        'urdf', 'mycobot_280_with_friction.urdf.xacro'
    )
    
    # Gazebo launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_mycobot_gazebo, 'launch', 'mycobot.gazebo.launch.py'])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'load_controllers': 'true',
            'use_camera': 'true',
            'world_file': world_file,
            'use_rviz': 'false',
            'z': z_position,
            # Add the custom URDF path
            'robot_description_file': custom_urdf_path
        }.items()
    )
    
    return LaunchDescription(
        declared_arguments + [
            gazebo_launch
        ]
    )

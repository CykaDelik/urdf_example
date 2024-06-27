import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    xacro_file_name = 'description/cy-borg.urdf.xacro'
    urdf_file_name = 'description/cy-borg.urdf'
    rviz_config_file_name = 'rviz/cy-borg_config.rviz'

    package_dir = get_package_share_directory('cy-borg')
    xacro_file = os.path.join(package_dir, xacro_file_name)
    urdf_file = os.path.join(package_dir, urdf_file_name)
    rviz_config_file = os.path.join(package_dir, rviz_config_file_name)

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),

        ExecuteProcess(
            cmd=['ros2', 'run', 'xacro', 'xacro', xacro_file, '-o', urdf_file],
            output='screen'),

        # Launch the robot state publisher first
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(package_dir, 'launch', 'rsp.launch.py')),
            launch_arguments={'use_sim_time': use_sim_time}.items()
        ),

        ExecuteProcess(
            cmd=['rviz2', '-d', rviz_config_file],
            output='screen')
    ])


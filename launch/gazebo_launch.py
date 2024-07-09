import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    xacro_file_name = 'description/cy-borg.urdf.xacro'
    urdf_file_name = 'description/cy-borg.urdf'
    controller_config = os.path.join(get_package_share_directory('cy-borg'), 'config', 'diff_drive_controller.yaml')
    package_dir = get_package_share_directory('cy-borg')
    xacro_file = os.path.join(package_dir, xacro_file_name)
    urdf_file = os.path.join(package_dir, urdf_file_name)

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),

        ExecuteProcess(
            cmd=['ros2', 'run', 'xacro', 'xacro', xacro_file, '-o', urdf_file],
            output='screen'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(package_dir, 'launch', 'rsp.launch.py')
            ),
            launch_arguments={'use_sim_time': use_sim_time}.items()
        ),

        ExecuteProcess(
            cmd=['gz', 'sim', 'empty.sdf', '-v', '4'],
            output='screen'
        ),

        ExecuteProcess(
            cmd=['gz', 'service', '-s', '/world/empty/create', '--reqtype', 'gz.msgs.EntityFactory', '--reptype', 'gz.msgs.Boolean', '--timeout', '1000',
                 '--req', f'sdf_filename: "{urdf_file}", name: "cy-borg", pose: {{position: {{x: 0, y: 0, z: 0.5}}, orientation: {{x: 0, y: 0, z: 0, w: 1}}}}'],
            output='screen'
        ),

        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[{'use_sim_time': use_sim_time},
                        os.path.join('/home/deck/dev_ws/src/cy-borg/config', 'diff_drive_controller.yaml')],
            output='screen'
        ),

        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=Node(
                    package='controller_manager',
                    executable='ros2_control_node',
                    parameters=[{'use_sim_time': use_sim_time},
                                os.path.join('/home/deck/dev_ws/src/cy-borg/config', 'diff_drive_controller.yaml')],
                    output='screen'
                ),
                on_start=[
                    Node(
                        package='controller_manager',
                        executable='spawner',
                        arguments=['diff_drive_controller'],
                        output='screen'
                    ),
                    Node(
                        package='controller_manager',
                        executable='spawner',
                        arguments=['joint_state_broadcaster'],
                        output='screen'
                    )
                ]
            )
        )
    ])
""" Launch as2_platform_dji_psdk node"""
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, EnvironmentVariable, PathJoinSubstitution
from launch import LaunchDescription


def generate_launch_description():
    """Entrypoint"""

    platform_config_file = PathJoinSubstitution([
        FindPackageShare('as2_platform_dji_psdk'),
        'config', 'platform_config_file.yaml'
    ])

    control_modes = PathJoinSubstitution([
        FindPackageShare('as2_multirotor_simulator'),
        'config', 'control_modes.yaml'
    ])

    return LaunchDescription([
        DeclareLaunchArgument('namespace',
                              default_value=EnvironmentVariable(
                                  'AEROSTACK2_SIMULATION_DRONE_ID'),
                              description='Drone namespace'),
        DeclareLaunchArgument('config_file',
                              default_value=platform_config_file,
                              description='Platform configuration file'),
        DeclareLaunchArgument('control_modes_file',
                              default_value=control_modes,
                              description='Platform control modes file'),
        Node(
            package="as2_platform_dji_psdk",
            executable="as2_platform_dji_psdk_node",
            name="as2_platform_dji_psdk",
            namespace=LaunchConfiguration('namespace'),
            output="screen",
            emulate_tty=True,
            parameters=[
                {
                    "control_modes_file": LaunchConfiguration('control_modes_file'),
                },
                LaunchConfiguration('config_file'),
            ]
        )
    ])

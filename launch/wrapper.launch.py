# Copyright (C) 2023 Unmanned Life
# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at http://mozilla.org/MPL/2.0/.


# This is a ROS2 launch file which starts the psdk_wrapper_node,
# configures it and activates it.

from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration
from launch.actions import EmitEvent, DeclareLaunchArgument
from launch_ros.actions import LifecycleNode
from launch_ros.events.lifecycle import ChangeState
from launch.substitutions import LaunchConfiguration, EnvironmentVariable, PathJoinSubstitution

import lifecycle_msgs.msg
import launch


def generate_launch_description():
    # Declare the namespace launch argument
    psdk_params_file = PathJoinSubstitution([
        FindPackageShare('as2_platform_dji_psdk'),
        'config', 'psdk_params.yaml'
    ])

    # Prepare the wrapper node
    wrapper_node = LifecycleNode(
        package="psdk_wrapper",
        executable="psdk_wrapper_node",
        name="psdk_wrapper_node",
        namespace=LaunchConfiguration('namespace'),
        output="screen",
        emulate_tty=True,
        parameters=[
                LaunchConfiguration('psdk_params'),
        ],
        remappings=[
            ("psdk_ros2/gps_position_fused", "sensor_measurements/gps"),
            ("psdk_ros2/imu", "sensor_measurements/imu")
        ]
    )

    # Configure lifecycle node
    wrapper_configure_trans_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(wrapper_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    # Activate lifecycle node
    wrapper_activate_trans_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(wrapper_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
        )
    )

    # Create LaunchDescription and populate
    ld = LaunchDescription([
        DeclareLaunchArgument('namespace',
                              default_value=EnvironmentVariable(
                                  'AEROSTACK2_SIMULATION_DRONE_ID'),
                              description='Drone namespace'),
        DeclareLaunchArgument('psdk_params',
                              default_value=psdk_params_file,
                              description='DJI PSDK configuration file')
    ])

    # Declare Launch options
    ld.add_action(wrapper_node)
    ld.add_action(wrapper_configure_trans_event)
    ld.add_action(wrapper_activate_trans_event)

    return ld

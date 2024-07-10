#!/usr/bin/env python3

"""Launch psdk_wrapper node."""

# Copyright 2023 Universidad Politécnica de Madrid
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the Universidad Politécnica de Madrid nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

__authors__ = 'Rafael Pérez Seguí'
__copyright__ = 'Copyright (c) 2022 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'
__version__ = '0.1.0'

import os

from ament_index_python.packages import get_package_share_directory
from as2_core.declare_launch_arguments_from_config_file import DeclareLaunchArgumentsFromConfigFile
from as2_core.launch_configuration_from_config_file import LaunchConfigurationFromConfigFile
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.events.lifecycle import ChangeState
import lifecycle_msgs.msg


def generate_launch_description() -> LaunchDescription:
    """
    Entry point for launch file.

    :return: Launch description
    :rtype: LaunchDescription
    """
    package_folder = get_package_share_directory(
        'as2_platform_dji_psdk')

    psdk_params_file = os.path.join(package_folder,
                                    'config/psdk_params.yaml')

    psdk_authentication_params_file = os.path.join(package_folder,
                                                   'config/psdk_authentication_params.yaml')

    link_config_file = os.path.join(package_folder,
                                    'config/link_config.json')

    hms_return_codes_file = os.path.join(package_folder,
                                         'config/hms_2023_08_22.json')

    # Prepare the wrapper node
    wrapper_node = LifecycleNode(
        package='psdk_wrapper',
        executable='psdk_wrapper_node',
        name='psdk_wrapper_node',
        namespace=LaunchConfiguration('namespace'),
        output='screen',
        emulate_tty=True,
        parameters=[
            {
                'link_config_file_path': LaunchConfiguration('link_config_file_path'),
                'hms_return_codes_path': LaunchConfiguration('hms_return_codes_path'),
            },
            LaunchConfiguration('psdk_authentication_params_file'),
            LaunchConfigurationFromConfigFile(
                'psdk_params_file',
                default_file=psdk_params_file),
        ],
        remappings=[
            ('psdk_ros2/gps_position_fused', 'sensor_measurements/gps'),
            ('psdk_ros2/imu', 'sensor_measurements/imu'),
            ('psdk_ros2/main_camera_stream',
             'sensor_measurements/main_camera/image_raw'),
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
        DeclareLaunchArgument('psdk_authentication_params_file',
                              default_value=psdk_authentication_params_file,
                              description='DJI PSDK authentication file'),
        DeclareLaunchArgument('link_config_file_path',
                              default_value=link_config_file,
                              description='DJI PSDK link configuration file'),
        DeclareLaunchArgument('hms_return_codes_path',
                              default_value=hms_return_codes_file,
                              description='Path to JSON file with known DJI return codes'),
        DeclareLaunchArgumentsFromConfigFile(
            name='psdk_params_file',
            source_file=psdk_params_file,
            description='Paremeters for DJI PSDK authentication'),
    ])

    # Declare Launch options
    ld.add_action(wrapper_node)
    ld.add_action(wrapper_configure_trans_event)
    ld.add_action(wrapper_activate_trans_event)

    return ld

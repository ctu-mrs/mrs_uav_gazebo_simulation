#!/usr/bin/env python3

import launch
import os

import rclpy.parameter

from launch_ros.actions import Node 
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
        LaunchConfiguration,
        IfElseSubstitution,
        PythonExpression,
        PathJoinSubstitution,
        EnvironmentVariable,
        )

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    ld = launch.LaunchDescription()

    pkg_name = "mrs_uav_gazebo_simulation"

    this_pkg_path = get_package_share_directory(pkg_name)
    node_name='mrs_drone_spawner'
    spawner_params = LaunchConfiguration('spawner_params')

    # #{ custom_config

    custom_config = LaunchConfiguration('custom_config')

    # this adds the args to the list of args available for this launch files
    # these args can be listed at runtime using -s flag
    # default_value is required to if the arg is supposed to be optional at launch time
    ld.add_action(DeclareLaunchArgument(
        'custom_config',
        default_value="",
        description="Path to the custom configuration file. The path can be absolute, starting with '/' or relative to the current working directory",
        ))

    # behaviour:
    #     custom_config == "" => custom_config: ""
    #     custom_config == "/<path>" => custom_config: "/<path>"
    #     custom_config == "<path>" => custom_config: "$(pwd)/<path>"
    custom_config = IfElseSubstitution(
            condition=PythonExpression(['"', custom_config, '" != "" and ', 'not "', custom_config, '".startswith("/")']),
            if_value=PathJoinSubstitution([EnvironmentVariable('PWD'), custom_config]),
            else_value=custom_config
            )

    # #} end of custom_config

    # #{ log_level

    ld.add_action(DeclareLaunchArgument(name='log_level', default_value='info'))

    # #} end of log_level

    ld.add_action(DeclareLaunchArgument(
        'spawner_params',
        default_value=os.path.join(this_pkg_path, 'config', 'spawner_params.yaml'),
        description='Path to the default spawner configuration file. The path can be absolute, starting with "/" or relative to the current working directory',
    ))
    

    ld.add_action(
            Node(
                # namespace=node_name,
                name=node_name,
                package=pkg_name,
                executable='mrs_drone_spawner.py',
                output="screen",
                arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],

                # prefix=['debug_roslaunch ' + os.ttyname(sys.stdout.fileno())],

                parameters=[
                    {'custom_config': custom_config},
                    spawner_params,
                    ],

                remappings=[
                ],
                )
            )

    return ld

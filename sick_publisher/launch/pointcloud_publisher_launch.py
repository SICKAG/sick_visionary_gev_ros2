#!/usr/bin/env python3
#
# Copyright (c) 2024 SICK AG, Waldkirch
# SPDX-License-Identifier: Unlicense

"""
This script is used to launch a ROS2 PointcloudPublisher-Node.
"""
from launch import LaunchDescription
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import LoadComposableNodes
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():

    pointcloudPub_name = LaunchConfiguration('node_name')
    pointcloudPub_ns = LaunchConfiguration('name_space')
    serial_number = LaunchConfiguration('serial_number')

    node_name_launch_arg = DeclareLaunchArgument(
        'node_name',
        default_value=''
    )

    name_space_launch_arg = DeclareLaunchArgument(
        'name_space',
        default_value=''
    )

    # This argument is currently always interpreted as integer see https://github.com/ros2/launch_ros/issues/384
    serial_number_launch_arg = DeclareLaunchArgument(
        'serial_number',
        default_value=''
    )

    pointcloud_node = ComposableNode(
        package='sick_visionary_gev_ros2',
        plugin='sick::PointCloudPublisher',
        parameters=[{'serial_number': serial_number}],
        namespace= pointcloudPub_ns,
        name=pointcloudPub_name)

    return LaunchDescription([LoadComposableNodes(target_container='/sick/visionary_cameras',
                                                  composable_node_descriptions=[pointcloud_node])])

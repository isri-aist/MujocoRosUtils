#!/usr/bin/env python3
##
# @file display.launch.py
#
# @brief Provide launch file for display node.
#
# @section author_doxygen_example Author(s)
# - Created by Masaki Murooka on 2024/11/29

# Standard library
import os

# External library
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer
from ament_index_python import get_package_share_directory
from launch.actions import DeclareLaunchArgument, OpaqueFunction


def launch_setup(context, *args, **kwargs):
    points = context.launch_configurations["points"]

    rviz_config_file = os.path.join(
        get_package_share_directory('mujoco_ros_utils'),
        'launch',
        'display.rviz')

    point_cloud_node = ComposableNodeContainer(
            name='nodelet_manager',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='depth_image_proc',
                    plugin='depth_image_proc::PointCloudXyzrgbNode',
                    name='point_cloud_xyzrgb',
                    remappings=[
                        ('rgb/camera_info', '/image/camera_info'),
                        ('rgb/image_rect_color', '/image/color'),
                        ('depth_registered/image_rect', '/image/depth'),
                        ('depth_registered/points', '/points'),
                    ],
                ),
            ],
            output='screen',
            condition=IfCondition(points)
        )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
    )

    return [
        point_cloud_node,
        rviz_node
    ]


def generate_launch_description():
    """! Generate launch description
    """
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "points",
            default_value="False",
            description="Whether to run point cloud node"
        )
    )

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)])

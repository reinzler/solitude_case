#!/usr/bin/env python

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess


def generate_launch_description():

    straight_flight_node = Node(
        package='px4_ros_com',
        executable='straight_flight',
        output='screen',
        shell=True,
    )

    aruco_detection = Node(
        package='px4_ros_com',
        executable='aruco_detection',
        output='screen',
        shell=True,
    )

    return LaunchDescription([
        aruco_detection,
        straight_flight_node,

    ])

import os
import yaml
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
)
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ariac_moveit_config.parameters import generate_parameters


def launch_setup(context, *args, **kwargs):
    """
    Function to set up the launch configuration for moving the robot.

    Args:
        context (LaunchContext): The launch context.
        *args: Additional positional arguments.
        **kwargs: Additional keyword arguments.

    Returns:
        List[Node]: A list of nodes to start for moving the robot.
    """
    demo_cpp = Node(
        package="final_2",
        executable="robot_ariac",
        output="screen",
        parameters=generate_parameters()+[{"use_sim_time":True}],
    )

    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                FindPackageShare("ariac_moveit_config"),
                "/launch",
                "/ariac_robots_moveit.launch.py",
            ]
        )
    )

    nodes_to_start = [
        demo_cpp,
        moveit
    ]

    return nodes_to_start


def generate_launch_description():
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz", default_value="false", description="start rviz node?"
        )
    )

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
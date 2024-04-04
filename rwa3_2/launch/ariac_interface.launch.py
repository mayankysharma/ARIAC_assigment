from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
# from ariac_moveit_config.parameters import generate_parameters


def generate_launch_description():
    '''
    Generates a LaunchDescription for launching the ARIAC interface node.

    Returns:
        LaunchDescription: The LaunchDescription instance.
    '''
    ld = LaunchDescription()
    # Interface node
    rwa3_interface = Node(
        package="rwa3_2",
        executable="ariac_interface.py",
        name="rwa3_interface",
        output="screen",
        # parameters=generate_parameters()
    )

    ld.add_action(rwa3_interface)
    return ld  # Return LaunchDescription instance

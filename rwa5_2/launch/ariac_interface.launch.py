from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from ariac_moveit_config.parameters import generate_parameters


def generate_launch_description():
    '''
    Generates a LaunchDescription for launching the ARIAC interface node.

    Returns:
        LaunchDescription: The LaunchDescription instance.
    '''
    ld = LaunchDescription()

    # Interface node
    rwa5_interface = Node(
        package="rwa5_2",
        executable="ariac_interface_main.py",
        name="rwa5_interface",
        output="screen",
        # parameters=generate_parameters()
    )

    robot_node = Node(
        package="rwa5_2",
        executable="floor_robot",
        output="screen",
        parameters=generate_parameters(),
    )

    # start_rviz = LaunchConfiguration("rviz")

    # rviz_config_file = PathJoinSubstitution(
    #     [FindPackageShare("rwa5_2"), "rviz", "moveit_demo.rviz"]
    # )

    # rviz_node = Node(
    #     package="rviz2",
    #     executable="rviz2",
    #     name="rviz2_moveit",
    #     output="log",
    #     arguments=["-d", rviz_config_file],
    #     parameters=generate_parameters(),
    #     condition=IfCondition(start_rviz),
    # )


    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                FindPackageShare("ariac_moveit_config"),
                "/launch",
                "/ariac_robots_moveit.launch.py",
            ]
        )
    )
    
    ld.add_action(rwa5_interface)
    # ld.add_action(robot_node)
    # ld.add_action(moveit)
    return ld  # Return LaunchDescription instance

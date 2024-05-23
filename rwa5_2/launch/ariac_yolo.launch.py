from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
# from ariac_moveit_config.parameters import generate_parameters


def generate_launch_description():
    '''
    Generates a LaunchDescription for launching the ARIAC Sensor YOLO node.

    Returns:
        LaunchDescription: The LaunchDescription instance.
    '''
    ld = LaunchDescription()
    # Interface node
    final_yolo1 = Node(
        package="rwa5_2",
        executable="yolo_combined.py",
        name="image_subscriber_7",
        output="screen",
        # parameters=generate_parameters()
    )
    final_yolo2 = Node(
    package="rwa5_2",
    executable="yolonode_leftbin.py",
    name="image_subscriber_2",
    output="screen",
    # parameters=generate_parameters()
    )
    final_yolo3 = Node(
    package="rwa5_2",
    executable="yolonode_tray1.py",
    name="image_subscriber_3",
    output="screen",
    # parameters=generate_parameters()
    )
    final_yolo4 = Node(
    package="rwa5_2",
    executable="yolonode_tray2.py",
    name="image_subscriber_4",
    output="screen",
    # parameters=generate_parameters()
    )
    
    ld.add_action(final_yolo1)
    ld.add_action(final_yolo2)
    ld.add_action(final_yolo3)
    ld.add_action(final_yolo4)
    return ld  # Return LaunchDescription instance
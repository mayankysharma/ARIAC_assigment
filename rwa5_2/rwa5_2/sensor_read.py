import os
import yaml
from functools import partial

import rclpy
from rclpy.qos import qos_profile_sensor_data

from ariac_msgs.msg import (
    Order as OrderMsg,
    AGVStatus as AGVStatusMsg,
    AssemblyTask as AssemblyTaskMsg,
    Part as PartMsg,
    AdvancedLogicalCameraImage as AdvancedLogicalCameraImageMsg,
    BasicLogicalCameraImage as BasicLogicalCameraImageMsg,
    PartPose as PartPoseMsg
)
from sensor_msgs.msg import (
    Image as ImageMsg,
    PointCloud as PointCloudMsg,
    LaserScan as LaserScanMsg,
)
from geometry_msgs.msg import Pose

from utils import (  
    Mult_pose,
    Quart_to_RPY,
    AdvancedLogicalCameraImage,
    COLOROFPARTS,
    TYPEOFPARTS,
    RPY_to_Quart
)
from launch_ros.substitutions import FindPackageShare
    

# Constants mapping ARIAC sensor types to their ROS message types
ARIAC_SENSORS_2_TYPE = {
    'rgb_camera': "rgb_image",
    'basic_logical_camera': "image",
    'advanced_logical_camera': "image",
}

# Constants mapping ARIAC sensor types to their ROS message classes
ARIAC_SENSORS_2_Msg = {
    'rgb_camera': ImageMsg,
    'basic_logical_camera': BasicLogicalCameraImageMsg,
    'advanced_logical_camera': AdvancedLogicalCameraImageMsg,
}

class SensorRead():
    def __init__(self, node, callback_group, sensor_config="sensors"):
        # Finding the package share directory
        pkg_share = FindPackageShare(package='rwa5_2').find('rwa5_2')
        sensor_config_path = os.path.join(pkg_share, 'config', sensor_config + ".yaml")
        
        # Initialize node and sensor data list
        self.node = node
        self.sensor_data = []

        # Load sensor configuration from YAML file
        with open(sensor_config_path, "r") as file:
            self.yaml_data = yaml.safe_load(file)
        
        # Initialize sensors_info dictionary
        self.sensors_info = {}
        
        # Create subscriptions for each sensor based on configuration
        for sensor_name, info in self.yaml_data["sensors"].items():
            self.sensors_info[sensor_name] = self.node.create_subscription(
                ARIAC_SENSORS_2_Msg[info["type"]],
                f"/ariac/sensors/{sensor_name}/{ARIAC_SENSORS_2_TYPE[info['type']]}",
                partial(self._advanced_camera_cb, name=sensor_name),
                qos_profile_sensor_data, callback_group=callback_group   
            )

        # Initialize sensor_data dictionary
        self.sensor_data = {}

    def _advanced_camera_cb(self, msg: AdvancedLogicalCameraImageMsg, name: str):
        '''
        Callback for the topic advanced_camera

        Arguments:
            msg -- AdvancedLogicalCameraImage message
            name -- Name of the sensor
        '''
        # Parse AdvancedLogicalCameraImage message
        _image = AdvancedLogicalCameraImage(msg.part_poses,
                                            msg.tray_poses,
                                            msg.sensor_pose)
        # Store parsed data in sensor_data dictionary
        self.sensor_data[name] = self.parse_advanced_camera_image(_image)
     
    def parse_advanced_camera_image(self, image: AdvancedLogicalCameraImage) -> str:
        '''
        Parse an AdvancedLogicalCameraImage message and return a string representation.
        '''

        # Initialize output list
        output = []
        
        # Process part poses
        for i, part_pose in enumerate(image._part_poses):
            part_world_pose = Mult_pose(image._sensor_pose, part_pose.pose)
            roll, pitch, yaw = Quart_to_RPY(part_world_pose.orientation)
            output.append({
                "is_part": True,
                "pose": [part_world_pose.position.x, part_world_pose.position.y, part_world_pose.position.z],
                "orientation": [roll, pitch, yaw],
                "color": part_pose.part.color,
                "type": part_pose.part.type,
                "tray_id": None
            })
        
        # Process tray poses
        for i, tray in enumerate(image._tray_poses):
            tray_world_pose = Mult_pose(image._sensor_pose, tray.pose)
            roll, pitch, yaw = Quart_to_RPY(tray_world_pose.orientation)
            output.append({
                "is_part": False,
                "pose": [tray_world_pose.position.x, tray_world_pose.position.y, tray_world_pose.position.z],
                "orientation": [roll, pitch, yaw],
                "color": None,
                "type": None,
                "tray_id": tray.id
            })

        # Return parsed output
        return output

    def get_part_pose_from_sensor(self, part_type, part_color, verbose = False):
        """
        Retreive the order part info from sensor data and which will be use to pass it to robot for further processing
        """
        # key : (type, color, pose)

        for sensor_name, sensor_data in self.sensor_data.items():
            for sdata in sensor_data:
                if sdata["is_part"]:
                    if sdata["type"]==part_type and sdata["color"] == part_color:
                        # Store the pose, tray_id and agv_num for processing the tray like pick and place
                        pose = Pose()
                        pose.position.x,pose.position.y, pose.position.z = sdata["pose"]
                        quart = RPY_to_Quart(sdata["orientation"])
                        (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w) = quart 
                        if verbose:
                            self.node.get_logger().info(f"""\n==========================
            - {COLOROFPARTS[sdata["color"]]} {TYPEOFPARTS[sdata["type"]]}
                - Position (xyz): [{sdata["pose"][0]:.3f}, {sdata["pose"][1]:.3f}, {sdata["pose"][2]:.3f}]
                - Orientation (rpy): [{sdata["orientation"][0]:.3f}, {sdata["orientation"][1]:.3f}, {sdata["orientation"][2]:.3f}]\n=============\n""")

                        return {
                                "type" : sdata["type"],
                                "color" : sdata["color"],
                                "pose" : pose,
                                "kts" : 2 if pose.position.y > 0 else 1,
                                "bin_side" : "right_bins" if pose.position.x < 0 else "left_bins"
                                }



    def get_tray_pose_from_sensor(self, tray_id, verbose= False):
        """
        Retreive the order part info from sensor data and which will be use to pass it to robot for further processing
        """
        # key : (type, color, pose)

        for sensor_name, sensor_data in self.sensor_data.items():
            for sdata in sensor_data:
                if not sdata["is_part"]:
                    if sdata["tray_id"]==tray_id:
                        # Store the pose, tray_id and agv_num for processing the tray like pick and place
                        pose = Pose()
                        pose.position.x,pose.position.y, pose.position.z = sdata["pose"]
                        quart = RPY_to_Quart(sdata["orientation"])
                        (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w) = quart 
                        if verbose:
                            self.node.get_logger().info(f"""\n==========================
                - ID : {tray_id}
                - Position (xyz): [{sdata["pose"][0]:.3f}, {sdata["pose"][1]:.3f}, {sdata["pose"][2]:.3f}]
                - Orientation (rpy): [{sdata["orientation"][0]:.3f}, {sdata["orientation"][1]:.3f}, {sdata["orientation"][2]:.3f}]\n=============\n""")
                        
                        return {
                            "tray_id" : tray_id,
                            "pose" : pose,
                            "kts" : 2 if pose.position.y > 0 else 1
                        }

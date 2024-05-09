#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
from matplotlib import pyplot as plt
from ariac_msgs.msg import AdvancedLogicalCameraImage, PartPose, KitTrayPose, BasicLogicalCameraImage
from geometry_msgs.msg import Pose, Point, Quaternion
from rclpy.qos import qos_profile_sensor_data
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
import math



class ImageSubscriber_1(Node):
    def __init__(self):
        super().__init__('image_subscriber_1')
        self.bridge = CvBridge()
        # Define QoS profile with desired reliability and durability
        self.callback_group = MutuallyExclusiveCallbackGroup()
        self.callback_group_2 = MutuallyExclusiveCallbackGroup()

        self.qos_profile = rclpy.qos.qos_profile_sensor_data

        self.publisher_ = self.create_publisher(AdvancedLogicalCameraImage, '/ariac/sensors/left_bins_camera_advanced_logical/image', 10)

        self.subscription = self.create_subscription(
            Image,
            '/ariac/sensors/left_bins_camera/rgb_image',  # Replace this with your image topic
            self.callback,10,callback_group=self.callback_group)
        self.model = YOLO("/home/mayank/ariac_ws/src/ARIAC_assigment/final_2/final_2/best.pt")
        self.classNames = ["blue_battery","blue_pump","blue_regulator","blue_sensor",
                           "green_battery","green_pump","green_regulator","green_sensor",
                           "orange_battery","orange_pump","orange_regulator","orange_sensor",
                           "purple_battery","purple_pump","purple_regulator","purple_sensor",
                           "red_battery","red_pump","red_regulator","red_sensor",]
        self.subscription_ = self.create_subscription(
            BasicLogicalCameraImage,
            '/ariac/sensors/left_bins_camera_logical/image',
            self.listener_callback,
            self.qos_profile,callback_group=self.callback_group_2
              )
        self.constants={
            "battery": 10,
            "pump": 11,
            "regulator": 13,
            "sensor": 12,
            "green": 1,
            "blue": 2,
            "red": 0,
            "orange": 3,
            "purple": 4,
 
        }
        self.partinformaton = {}
        
    def listener_callback(self, msg):
        print('Received message:', msg)
        publish_msg=AdvancedLogicalCameraImage()
        self.get_logger().info('Received message:')
        self.received_part_poses = msg.part_poses
        self.received_tray_poses = msg.tray_poses
        self.received_sensor_pose = msg.sensor_pose
        self.get_logger().info('Data saved.')
        for i, part_pose in enumerate(self.received_part_poses):
            x, y, z = part_pose.position.x, part_pose.position.y, part_pose.position.z
            print('x:', x, 'y:', y, 'z:', z)
            self.map_coord = self.map_coordinates(round(y, 2), round(z, 2))
            print('map_coordinates',self.map_coord)
            min_distance = float('inf')
            for key in self.partinformaton.keys():
                    centroid_x, centroid_y = key
                    distance = math.sqrt((centroid_x - self.map_coord[0])**2 + (centroid_y - self.map_coord[1])**2)
                    if distance < min_distance:
                        min_distance = distance
                        self.nearest_key = key
            print("self.partinformaton",self.partinformaton)
            if self.partinformaton =={}:
                self.flag=True
                continue
            part_pose_pub = PartPose()
            part_pose_pub.part.color= self.constants[self.partinformaton[self.nearest_key][0]]
            part_pose_pub.part.type = self.constants[self.partinformaton[self.nearest_key][1]]
            part_pose_pub.pose.position.x = part_pose.position.x
            part_pose_pub.pose.position.y = part_pose.position.y
            part_pose_pub.pose.position.z = part_pose.position.z
            part_pose_pub.pose.orientation.x = part_pose.orientation.x
            part_pose_pub.pose.orientation.y = part_pose.orientation.y
            part_pose_pub.pose.orientation.z = part_pose.orientation.z
            part_pose_pub.pose.orientation.w = part_pose.orientation.w
            publish_msg.part_poses.append(part_pose_pub)
            publish_msg.tray_poses = self.received_tray_poses
            publish_msg.sensor_pose = self.received_sensor_pose
        self.flag=False    
        if not self.flag:
            self.publisher_.publish(publish_msg)
            self.get_logger().info('Message published.')

    def callback(self, msg):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.width, self.height = self.cv_image.shape[1], self.cv_image.shape[0]
        except Exception as e:
            self.get_logger().error('Error converting image: %s' % str(e))
            return

        self.optical_flow()

    def map_coordinates(self,x, y):
        x_min, x_max = -0.67, 0.43
        y_min, y_max = -0.56, 0.54

        target_x_min, target_x_max =514,142
        target_y_min, target_y_max = 426,62

        mapped_x = int((x - x_min) / (x_max - x_min) * (target_x_max - target_x_min) + target_x_min)
        mapped_y = int((y - y_min) / (y_max - y_min) * (target_y_max - target_y_min) + target_y_min)

        return((mapped_x, mapped_y))

    def optical_flow(self):
        results = self.model(self.cv_image, stream=True)

        for r in results:
            boxes = r.boxes

            for box in boxes:
                x1, y1, x2, y2 = box.xyxy[0]
                x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
                centroid_x = (x1 + x2) // 2
                centroid_y = (y1 + y2) // 2

                confidence = box.conf[0]

                if confidence > 0.7:
                    cv2.rectangle(self.cv_image, (x1, y1), (x2, y2), (255, 0, 255), 3)
                    print("centroid_x",centroid_x)
                    print("centroid_y",centroid_y)
                    cls = int(box.cls[0])
                    print("Class name -->", self.classNames[cls], "centroid_x",centroid_x, "centroid_y",centroid_y)
                    class_name_parts = self.classNames[cls].split('_')
                    self.partinformaton[(centroid_x,centroid_y)] = class_name_parts

                    org = [x1, y1]
                    font = cv2.FONT_HERSHEY_SIMPLEX
                    fontScale = 1
                    color = (255, 0, 0)
                    thickness = 2

                    cv2.putText(self.cv_image, self.classNames[cls], org, font, fontScale, color, thickness)

            cv2.imshow("yolo", self.cv_image)
            cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber_1()
    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

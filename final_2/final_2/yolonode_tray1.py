#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
import numpy as np
from ultralytics import YOLO
from matplotlib import pyplot as plt
from ariac_msgs.msg import AdvancedLogicalCameraImage, PartPose, KitTrayPose, BasicLogicalCameraImage
from geometry_msgs.msg import Pose, Point, Quaternion
from rclpy.qos import qos_profile_sensor_data
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import math



class ImageSubscriber_3(Node):
    def __init__(self):
        super().__init__('image_subscriber_3')
        self.bridge = CvBridge()
        # Define QoS profile with desired reliability and durability
        self.callback_group = MutuallyExclusiveCallbackGroup()
        self.callback_group_2 = MutuallyExclusiveCallbackGroup()

        self.qos_profile = rclpy.qos.qos_profile_sensor_data

        self.publisher_ = self.create_publisher(AdvancedLogicalCameraImage, '/ariac/sensors/kts1_camera_advanced_logical/image', 10)

        self.subscription = self.create_subscription(
            Image,
            '/ariac/sensors/kts1_camera/rgb_image',  # Replace this with your image topic
            self.callback,10,callback_group=self.callback_group)
        self.subscription_ = self.create_subscription(
            BasicLogicalCameraImage,
            '/ariac/sensors/kts1_camera_logical/image',
            self.listener_callback,
            self.qos_profile,callback_group=self.callback_group_2
              )
        self.partinformaton = {}
        self.flag=True
        self.recieved_tray_poses = []
        self.recieved_part_poses = []
        self.recieved_sensor_pose = Pose()
        
    def listener_callback(self, msg):
        print('recieved message:')
        self.get_logger().info('recieved message:')
        self.recieved_part_poses = msg.part_poses
        self.recieved_tray_poses = msg.tray_poses
        self.recieved_sensor_pose = msg.sensor_pose
        self.get_logger().info('Data saved.')
        self.publish_msg=AdvancedLogicalCameraImage()
        
        for i, part_pose in enumerate(self.recieved_tray_poses):
            x, y, z = part_pose.position.x, part_pose.position.y, part_pose.position.z
            print('x:', x, 'y:', y, 'z:', z)
            self.map_coord = self.map_coordinates(round(y, 2), round(z, 2))
            print('map_coordinates',self.map_coord)
            min_distance = float('inf')
            for key in self.partinformaton.keys():
                    centroid_x = key
                    distance = abs(centroid_x - self.map_coord)
                    if distance < min_distance:
                        min_distance = distance
                        self.nearest_key = key
                    print("self.partinformaton",self.partinformaton)

                
                    tray_pose = KitTrayPose()
                    tray_pose.id = int(self.partinformaton[self.nearest_key])
                    tray_pose.pose.position.x = part_pose.position.x
                    tray_pose.pose.position.y = part_pose.position.y
                    tray_pose.pose.position.z = part_pose.position.z
                    tray_pose.pose.orientation.x = part_pose.orientation.x
                    tray_pose.pose.orientation.y = part_pose.orientation.y
                    tray_pose.pose.orientation.z = part_pose.orientation.z
                    tray_pose.pose.orientation.w = part_pose.orientation.w
                    self.publish_msg.tray_poses.append(tray_pose)

                    #publish_msg.part_poses.append(part_pose_pub)
                    self.publish_msg.part_poses = self.recieved_part_poses
                    self.publish_msg.sensor_pose = self.recieved_sensor_pose
                    self.flag=False   
            
        if not self.flag:
            self.publisher_.publish(self.publish_msg)
            self.get_logger().info('Message published.')
        else :
            self.publish_msg.tray_poses=[]
            self.publish_msg.part_poses = []
            self.publish_msg.sensor_pose = self.recieved_sensor_pose
            self.publisher_.publish(self.publish_msg)

    def callback(self, msg):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.width, self.height = self.cv_image.shape[1], self.cv_image.shape[0]
            height, width = self.cv_image.shape[:2]
            self.cv_image = cv2.resize( self.cv_image, (4*width, 4*height), interpolation=cv2.INTER_LINEAR)

            # cv2.imshow("Callback Image", self.cv_image)
            # cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error('Error converting image: %s' % str(e))
            return

        self.optical_flow()

    def map_coordinates(self,x, y):
        x_min, x_max = -0.52, 0.33
       

        target_x_min, target_x_max =458,180
        

        mapped_x = int((x - x_min) / (x_max - x_min) * (target_x_max - target_x_min) + target_x_min)

        return mapped_x
    
    def optical_flow(self):
        
       
            dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
            parameters =  cv2.aruco.DetectorParameters()
            detector = cv2.aruco.ArucoDetector(dictionary, parameters)
            
            

            # Detect the markers in the grayscale image
            corners, ids, rejectedImgPoints =  detector.detectMarkers( self.cv_image)
        # Draw the ArUco markers on the original color image
            self.cv_image = aruco.drawDetectedMarkers(self.cv_image, corners, ids, borderColor=(255, 0, 0))
            print("ID",ids)
            
            self.partinformaton = {}
            for i in range(len(ids)):
                # Calculate centroid of the first detected marker
                centroid_x = int(np.mean(corners[i][0][:, 0])/4)
                centroid_y = int(np.mean(corners[i][0][:, 1])/4)
                # Print the ArUco marker ID and centroid coordinates
                print('Detected ArUco Marker ID: %s, Centroid X: %d, Y: %d' % (ids[i][0], centroid_x, centroid_y))
                for key in self.partinformaton.keys():
                    if abs(centroid_x - key) <= 20:
                        break
                else:
                    self.partinformaton[centroid_x] = ids[i][0]
                
                # Display the image with detected markers
            
            print('part information',self.partinformaton)

            


def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber_3()
    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

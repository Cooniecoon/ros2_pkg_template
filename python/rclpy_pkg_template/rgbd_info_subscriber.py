#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
import message_filters
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
from image_geometry import cameramodels

import cv2
import numpy as np



class ImageSubscriber(Node):
	def __init__(self):
		super().__init__('astra_ground_normal_calibration_node')
		self.declare_parameter("camera_name", "front_bottom")
		self.camera_name = self.get_parameter("camera_name").value

		self.rgb_sub = message_filters.Subscriber(self, Image, f'/{self.camera_name}/color/image_raw', qos_profile=qos_profile_sensor_data)
		self.depth_sub = message_filters.Subscriber(self, Image, f'/{self.camera_name}/depth/image_raw', qos_profile=qos_profile_sensor_data)

		self.rgb_info_sub = self.create_subscription(CameraInfo, f'/{self.camera_name}/color/camera_info', self.rgb_info_callback, qos_profile_sensor_data)
		self.depth_info_sub = self.create_subscription(CameraInfo, f'/{self.camera_name}/depth/camera_info', self.depth_info_callback, qos_profile_sensor_data)
		# self.ts = message_filters.TimeSynchronizer([self.rgb_sub, self.depth_sub], 30)
		queue_size = 30
		self.ts = message_filters.ApproximateTimeSynchronizer([self.rgb_sub, self.depth_sub], queue_size, 0.01)
		self.ts.registerCallback(self.image_callback)

		self.cv_bridge = CvBridge()

		self.is_rgb_info_received = False
		self.is_depth_info_received = False
		self.rgb_camera_info = cameramodels.PinholeCameraModel()
		self.rgb_K = np.eye(3)
		self.rgb_D = np.zeros((1, 4))

		self.depth_camera_info = cameramodels.PinholeCameraModel()
		self.depth_K = np.eye(3)
		self.depth_D = np.zeros((1, 4))


	def image_callback(self, rgb_msg, depth_msg):
		if self.is_rgb_info_received and self.is_depth_info_received:
			rgb_image = self.cv_bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
			depth_image = self.cv_bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
            
			cv2.imshow('RGB Image', rgb_image)
			cv2.imshow('Depth Image', depth_image)
			cv2.waitKey(3)

	def rgb_info_callback(self,msg):
		if self.is_rgb_info_received == False:
			self.rgb_camera_info.fromCameraInfo(msg)
			self.rgb_K = np.array(self.rgb_camera_info.full_K)
			self.rgb_D = np.array(self.rgb_camera_info.D)
			self.is_rgb_info_received=True

	def depth_info_callback(self,msg):
		if self.is_depth_info_received == False:
			self.depth_camera_info.fromCameraInfo(msg)
			self.depth_K = np.array(self.depth_camera_info.full_K)
			self.depth_D = np.array(self.depth_camera_info.D)
			self.is_depth_info_received=True
		

def main(args=None):
	rclpy.init(args=args)
	image_subscriber = ImageSubscriber()
	rclpy.spin(image_subscriber)
	rclpy.shutdown()

if __name__ == '__main__':
	main()

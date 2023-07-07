import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import message_filters
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge

import cv2

class ImageSubscriber(Node):
	def __init__(self):
		super().__init__('image_subscriber')
		self.rgb_sub = message_filters.Subscriber(self, Image, '/image_topic1', qos_profile=qos_profile_sensor_data)
		self.depth_sub = message_filters.Subscriber(self, Image, '/image_topic2', qos_profile=qos_profile_sensor_data)
		# self.ts = message_filters.TimeSynchronizer([self.rgb_sub, self.depth_sub], 10)
		queue_size = 30
		self.ts = message_filters.ApproximateTimeSynchronizer([self.rgb_sub, self.depth_sub], queue_size, 0.01)
		self.ts.registerCallback(self.image_callback)

		self.cv_bridge = CvBridge()

	def image_callback(self, rgb_msg, depth_msg):
		rgb_image = self.cv_bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
		depth_image = self.cv_bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')

		cv2.imshow('RGB Image', rgb_image)
		cv2.imshow('Depth Image', depth_image)
		cv2.waitKey(3)

def main(args=None):
	rclpy.init(args=args)
	image_subscriber = ImageSubscriber()
	rclpy.spin(image_subscriber)
	rclpy.shutdown()

if __name__ == '__main__':
	main()

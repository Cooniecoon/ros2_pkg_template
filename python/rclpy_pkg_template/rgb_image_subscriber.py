import rclpy 
from rclpy.qos import qos_profile_sensor_data
from rclpy.node import Node 
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge
import cv2 
 
class RgbImageSubscriber(Node):
  def __init__(self):
    super().__init__('image_subscriber')
    self.subscription = self.create_subscription(Image, '/image_raw', self.listener_callback, qos_profile_sensor_data)
    self.cv_bridge = CvBridge()
   
  def listener_callback(self, data):
    self.get_logger().info('Receiving video frame')
    current_frame = self.cv_bridge.imgmsg_to_cv2(data, "bgr8")
    cv2.imshow("camera", current_frame)
    cv2.waitKey(1)
  
def main(args=None):

    rclpy.init(args=args)
    image_subscriber = RgbImageSubscriber()
    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node()
    rclpy.shutdown()
  
if __name__ == '__main__':
  main()
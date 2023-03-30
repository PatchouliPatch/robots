import cv2

from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import time
import numpy as np

class ImagePublisher(Node):
	def __init__(self):

		super().__init__('image_publisher')

		qos_profile = QoSProfile( # credit to: https://answers.ros.org/question/360676/qos-python-code-for-create_subscriber/
		reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
		history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
		depth=1
		)

		self.publisher = self.create_publisher(CompressedImage, 'image', qos_profile=qos_profile)
		self.br = CvBridge()

		timer_period = 1/15 # 15 frames per second
		self.timer = self.create_timer(timer_period, self.timer_callback)
		self.camera_frame = 0
		self.camera = cv2.VideoCapture(0)
		self.camera.set(3, 256) # height = 256
		self.camera.set(4, 256) # width = 256
		self.camera.set(5, 15) # set to 15 to avoid congesting network
		self.enable_view = 0
		self.set_params()
		self.get_params()

	def set_params(self):
		self.declare_parameter('enable_view',0)

	def get_params(self):
		self.enable_view = self.get_parameter('enable_view').get_parameter_value().integer_value

	def timer_callback(self):

		# get the image data

		result, image = self.camera.read()
		if result:
			image = image.astype(np.uint8) # change to uint8
			self.get_logger().info(f"[CAMERA ACTIVE]", throttle_duration_sec=5) # stop printing every 5 seconds
			msg = self.br.cv2_to_compressed_imgmsg(image)
			msg.header.stamp = self.get_clock().now().to_msg()
			if self.enable_view:
				cv2.imshow("CAMERA",image)
			self.publisher.publish(msg)

def main(args=None):
	rclpy.init(args=args)

	image_publisher = ImagePublisher()
	rclpy.spin(image_publisher)
	image_publisher.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()

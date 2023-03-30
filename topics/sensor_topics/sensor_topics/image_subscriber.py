import cv2

from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge

import rclpy
import time
import datetime
import numpy as np

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

class ImageSubscriber(Node):

	def __init__(self):

		super().__init__('image_subscriber')

		qos_profile = qos_profile_sensor_data

		self.enable_display = 1
		self.br = CvBridge()
		self.create_subscription(CompressedImage, 'image', self.imgsub_callback, qos_profile)
		self.frames = 0
		self.curr_time = time.time()

	def imgsub_callback(self, msg=None):

		if msg != None:

			d_time = time.time() - self.curr_time
			self.curr_time = time.time()
			self.get_logger().info(f"[Camera]: Framrate: {1/d_time}")

			if self.enable_display:
				image = self.br.compressed_imgmsg_to_cv2(msg)
				cv2.imshow("Image Subscriber", image.astype(np.uint8))

			# do other useful things here.

def main(args=None):

	rclpy.init(args=args)

	image_subscriber = ImageSubscriber()
	rclpy.spin(image_subscriber)
	image_subscriber.destroy_node()

	rlcpy.shutdown()

if __name__ == '__main__':
	main()

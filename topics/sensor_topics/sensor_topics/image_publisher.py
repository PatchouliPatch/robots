
import cv2
import asyncio
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import time
import numpy as np
from collections import deque

class ImagePublisher(Node):
	def __init__(self):

		super().__init__('image_publisher')
		print("Starting Image Publisher...")

		qos_profile = QoSProfile( # credit to: https://answers.ros.org/question/360676/qos-python-code-for-create_subscriber/
		reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
		history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
		depth=1
		)

		self.publisher = self.create_publisher(CompressedImage, 'image', qos_profile=qos_profile)
		self.br = CvBridge()

		self.timer_publisher = self.create_timer(1/30, self.process_image)
		self.timer_camera = self.create_timer(1/30, self.video_capture)
		self.timer_fps = self.create_timer(1,self.fps)
		self.camera_frame = 0
		self.get_sample_frame = 0
		self.gray_scale = 0
		self.camera = cv2.VideoCapture(0)
		self.camera.set(3, 640) # height = 256
		self.camera.set(4, 480) # width = 256
		self.camera.set(5, 30) # set max fps to 30
		self.enable_view = 0
		self.create_sample_video = 0
		self.current_time = time.time()
		self.set_params()
		self.get_params()
		self.frame_storage = deque([])
		self.imgmsg_storage = deque([])
		self.last_time = time.time()
		self.dt_array = deque([])
		self.captured_frames = 0
		self.published_frames = 0

	def set_params(self):
		self.declare_parameter('enable_view',0)
		self.declare_parameter('get_sample_frame',0)
		self.declare_parameter('gray_scale',0)

	def get_params(self):
		self.enable_view = self.get_parameter('enable_view').get_parameter_value().integer_value
		self.get_sample_frame = self.get_parameter('get_sample_frame').get_parameter_value().integer_value
		self.gray_scale = self.get_parameter('gray_scale').get_parameter_value().integer_value

	async def video_capture(self):

		captured, frame = self.camera.read()

		if captured:
			if self.gray_scale:
				frame = cv2_cvtColor(frame, cv2.COLOR_BGR2GRAY)
			self.frame_storage.append(frame)
			self.captured_frames += 1
			return frame

	async def process_image(self):

		if len(self.frame_storage) > 0:
			msg = self.br.cv2_to_compressed_imgmsg(self.frame_storage.popleft())
			msg.header.stamp = self.get_clock().now().to_msg()
			self.publisher.publish(msg)
			self.published_frames += 1

	def fps(self): #called once a second

		self.get_logger().info(f"FPS: {self.captured_frames} {len(self.frame_storage)} | Published: {self.published_frames}")
		self.captured_frames, self.published_frames = 0, 0

def main(args=None):
	rclpy.init(args=args)

	image_publisher = ImagePublisher()
	rclpy.spin(image_publisher)
	image_publisher.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()

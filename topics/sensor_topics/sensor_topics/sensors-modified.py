
import rclpy
import time
import datetime
import numpy as np

from rclpy.node import Node
from sensor_msgs.msg import Imu, BatteryState
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data

class SensorsSubscriber(Node):
	def __init__(self):
		super().__init__('sensors_subscriber')
		
		self.scan_ranges = []
		self.init_scan_state = False

		# subscription format: create_subscription(NameOfCallback, 'topic_name', self.listener_callback, qos_profile)
		self.create_subscription(Imu, 'imu', self.imu_callback, qos_profile_sensor_data)
		self.create_subscription(BatteryState, 'battery_state', self.battery_callback, qos_profile=qos_profile_sensor_data)
		self.create_subscription(LaserScan, 'scan', self.scan_callback, qos_profile=qos_profile_sensor_data)
		self.imu_count = 0

	def imu_callback(self, msg):
		orientation_covariance_vector = msg.orientation_covariance

		self.get_logger().info(f"Orientation: {msg.orientation_covariance}| AV: {msg.angular_velocity} | LAccel: {msg.linear_acceleration}")
		time.sleep(1) # delay by 1 second

	def scan_callback(self, msg):
		self.get_logger().info(f"Scan Min Range: {msg.range_min} | Scan Max Range: {msg.range_max}")
		time.sleep(1)
	
	def battery_callback(self, msg):
		self.get_logger().info(f"Battery: {msg.present} | Voltage: {msg.voltage} | Temp: {msg.temperature} | Current: {msg.current}")
		time.sleep(1)
def main(args=None):

	rclpy.init(args=args)

	sensors_subscriber = SensorsSubscriber()
	rclpy.spin(sensors_subscriber)
	sensors_subscriber.destroy_node()

if __name__ == '__main__':

	main()

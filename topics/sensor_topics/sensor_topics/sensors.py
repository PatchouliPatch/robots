
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

		abt_x = msg.orientation.x
		abt_y = msg.orientation.y
		abt_z = msg.orientation.z

		av_x, av_y, av_z = msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z
		la_x, la_y, la_z = msg.linear_acceleration.x, msg.angular_velocity.y, msg.angular_velocity.z
		
		print(f'[{time.time()}] [Imu]: Orient ({abt_x},{abt_y},{abt_z}) | AV: ({av_x},{av_y},{av_z}) | LA: ({la_x},{la_y},{la_z})')
		time.sleep(1)
		# self.get_logger().info(f"Orientation: {msg.orientation_covariance}| AV: {msg.angular_velocity} | LAccel: {msg.linear_acceleration}")

	def scan_callback(self, msg):
	
		self.scan_ranges = []
		for i in msg.ranges: #filter some of the noise
			if i == 0:
				continue
			else:
				self.scan_ranges.append(i)
				
		self.scan_ranges = np.array(self.scan_ranges)

		print(f'[{time.time()}] [Scanner]: Min {self.scan_ranges.min()} | Max {self.scan_ranges.max()}')
		time.sleep(1)
		# self.get_logger().info(f"Scan Min Range: {msg.range_min} | Scan Max Range: {msg.range_max}")
	
	def battery_callback(self, msg):
		print(f'[{time.time()}] [Battery]: Bat: {msg.present} | Volt: {msg.voltage} | Temp: {msg.temperature} | Amps: {msg.current}')
		time.sleep(1)
		# self.get_logger().info(f"Battery: {msg.present} | Voltage: {msg.voltage} | Temp: {msg.temperature} | Current: {msg.current}")

def main(args=None):

	rclpy.init(args=args)

	sensors_subscriber = SensorsSubscriber()
	rclpy.spin(sensors_subscriber)
	sensors_subscriber.destroy_node()
	
	rclpy.shutdown()

if __name__ == '__main__':

	main()


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

		# store last variables for easy access later
		self.current_imu_msg = None
		self.current_scn__msg = None
		self.current_bat_msg = None

		# parameter list for easy reference
		self.SensorSubscriberParameterNames = (
			'imu_hz','scn_hz','bat_hz','imu_en','scn_en','bat_en'
		)
		# message frequency
		self.imu_hz = 1
		self.scn_hz = 1
		self.bat_hz = 1
		self.hz_debug = time.time()
		# enable/disable printing of variables
		self.imu_en = 1
		self.scn_en = 1
		self.bat_en = 1

		# set and get parameters
		self.set_params()
		self.get_params()

		# subscription format: create_subscription(NameOfCallback, 'topic_name', self.listener_callback, qos_profile)
		self.create_subscription(Imu, 'imu', self.imu_callback, qos_profile_sensor_data)
		self.create_subscription(BatteryState, 'battery_state', self.battery_callback, qos_profile=qos_profile_sensor_data)
		self.create_subscription(LaserScan, 'scan', self.scan_callback, qos_profile=qos_profile_sensor_data)
		self.imu_count = 0

		# Creating timers to make sure that they dont print continuously
		self.timer_imu = self.create_timer(self.imu_hz, self.imu_callback)
		#self.timer_imu.duration = self.imu_hz

		self.timer_scn = self.create_timer(self.scn_hz, self.scan_callback)
		#self.timer_scn.duration = self.scn_hz

		self.timer_bat = self.create_timer(self.bat_hz, self.battery_callback)
		#self.timer_bat.duration = self.bat_hz

	def set_params(self):

		for thing in self.SensorSubscriberParameterNames:
			self.declare_parameter(thing,1)

	def get_params(self):

		self.imu_hz = self.get_parameter('imu_hz').get_parameter_value().integer_value
		self.scn_hz = self.get_parameter('scn_hz').get_parameter_value().integer_value
		self.bat_hz = self.get_parameter('bat_hz').get_parameter_value().integer_value
		self.imu_en = self.get_parameter('imu_en').get_parameter_value().integer_value
		self.scn_en = self.get_parameter('scn_en').get_parameter_value().integer_value
		self.bat_en = self.get_parameter('bat_en').get_parameter_value().integer_value

	def imu_callback(self, msg=None):
		
		if msg != None:
			self.current_imu_msg = msg
		else:
			return

		if self.imu_en:
			abt_x = msg.orientation.x
			abt_y = msg.orientation.y
			abt_z = msg.orientation.z

			av_x, av_y, av_z = msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z
			la_x, la_y, la_z = msg.linear_acceleration.x, msg.angular_velocity.y, msg.angular_velocity.z

			self.get_logger().info(f'[Imu]: Orient ({abt_x},{abt_y},{abt_z}) | AV: ({av_x},{av_y},{av_z}) | LA: ({la_x},{la_y},{la_z})', throttle_duration_sec=self.imu_hz)

	def scan_callback(self, msg=None):

		if msg != None:
			self.current_scn_msg = msg
		else:
			return

		if self.scn_en:
			self.scan_ranges = []
			for i in msg.ranges: #filter some of the noise
				if i == 0:
					continue
				else:
					self.scan_ranges.append(i)
					
			self.scan_ranges = np.array(self.scan_ranges)

			self.get_logger().info(f'[Scanner]: Min {self.scan_ranges.min()} | Max {self.scan_ranges.max()}', throttle_duration_sec=self.scn_hz)

	def battery_callback(self, msg=None):

		if msg != None: 
			self.current_bat_msg = msg
		else:
			return

		if self.bat_en:
			self.get_logger().info(f'[Battery]: Bat: {msg.present} | Volt: {msg.voltage} | Temp: {msg.temperature} | Amps: {msg.current}', throttle_duration_sec=self.bat_hz)


def main(args=None):

	rclpy.init(args=args)

	sensors_subscriber = SensorsSubscriber()
	rclpy.spin(sensors_subscriber)
	sensors_subscriber.destroy_node()
	
	rclpy.shutdown()

if __name__ == '__main__':

	main()

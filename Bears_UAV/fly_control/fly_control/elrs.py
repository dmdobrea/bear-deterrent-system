#!/usr/bin/env python3

import sys
import serial
from threading import Lock
import time

import geometry_msgs.msg
import rclpy
import std_msgs.msg

from rclpy.node  import Node
from rclpy.qos   import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

IntToFloat = 23860929.0

class xbox (Node):

	def __init__(self):
		super().__init__('poc_xbox')
        
		qos_profile = QoSProfile(
			reliability=QoSReliabilityPolicy.BEST_EFFORT,
			durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
			history=QoSHistoryPolicy.KEEP_LAST,
			depth=1
			)
			
		#===============================================================================================
		# publisher nodes
		#
		self.stick_pub    = self.create_publisher (geometry_msgs.msg.Twist, '/offboard_velocity_cmd', qos_profile)
		self.actions_pub  = self.create_publisher (std_msgs.msg.UInt8, 	    '/action_message', 	      qos_profile)

		#================================== 
		# create timers
		#
		timer_period = 0.02  # seconds
		self.timer = self.create_timer(timer_period, self.xbox_loop_callback)

		timer_period2 = 0.04
		self.timer2 = self.create_timer(timer_period2, self.publisher_callback)
		
		#================================== 
		# init & configure
		#
		self.ser = serial.Serial('/dev/ttyELRS', 4800, timeout=1)

		#================================== 
		# variable & classes
		#
		# self.joystick    = pygame.joystick.Joystick(0)

		self.mutex = Lock()
		
		self.actions_msg = std_msgs.msg.UInt8()
		self.twist       = geometry_msgs.msg.Twist()
		self.yaw_val     = 0

		self.buttonX = 0
		self.buttonA = 0
		self.buttonB = 0
		self.buttonY = 0
		self.sync = 0
		self.message = list(0 for self.temp in range(11))

		self.x_val   = 0.0
		self.y_val   = 0.0
		self.z_val   = 0.0
		self.yaw_val = 0.0

		self.longitude = 0.0
		self.latitude  = 0.0
		
		self.mode = -1
		
		self.m3_vs_m2    = 1  	# if {m3_vs_m2 = 1} => mode 3 is active
					# if {m3_vs_m2 = 0} => mode 2 is active
					
					
	
	def publisher_callback(self):
		if self.mode == 0:
			self.mutex.acquire()
			self.twist.linear.x = self.x_val
			self.twist.linear.y = self.y_val
			self.twist.linear.z = self.z_val
			self.twist.angular.x = 0.0
			self.twist.angular.y = 0.0
			self.twist.angular.z = self.yaw_val	
			
			self.stick_pub.publish(self.twist)
			self.mutex.release()

	def xbox_loop_callback(self):
		while(self.sync != 2):
			self.x = int.from_bytes(self.ser.read(), byteorder='big')
			if(self.x == 0x55):
				self.sync += 1
			else:
				self.sync = 0
		
		if self.sync == 2:
			self.x = int.from_bytes(self.ser.read(), byteorder='big')
			if(self.x == 0x55):
				self.x = int.from_bytes(self.ser.read(), byteorder='big')
			self.sync = 0
			self.message[0] = self.x

			for self.i in range(1, 11):
				self.x = int.from_bytes(self.ser.read(), byteorder='big')
				self.message[self.i] = self.x

			self.check = self.message[0]
			for self.i in range(1, 10):
				self.check = self.check ^ self.message[self.i]

			self.messageIntegrity = True
			if self.check != self.message[10]:
				self.messageIntegrity = False

			if self.message[9] & 0x80 and self.message[1] != 0x56:
				self.messageIntegrity = False
			if self.message[9] & 0x40 and self.message[2] != 0x56:
				self.messageIntegrity = False
			if self.message[9] & 0x20 and self.message[3] != 0x56:
				self.messageIntegrity = False
			if self.message[9] & 0x10 and self.message[4] != 0x56:
				self.messageIntegrity = False
			if self.message[9] & 0x08 and self.message[5] != 0x56:
				self.messageIntegrity = False
			if self.message[9] & 0x04 and self.message[6] != 0x56:
				self.messageIntegrity = False
			if self.message[9] & 0x02 and self.message[7] != 0x56:
				self.messageIntegrity = False
			if self.message[9] & 0x01 and self.message[8] != 0x56:
				self.messageIntegrity = False
			
			if self.messageIntegrity:
				if False:
					for self.i in range(11):
						if self.message[self.i] > 127:
							print(self.message[self.i] - 256, end=' ')
						else:
							print(self.message[self.i], end=' ')
					print()

				self.mode = self.message[0] >> 6
				self.buttonX = (self.message[0] & (0x8))
				self.buttonB = (self.message[0] & (0x4))
				self.buttonA = (self.message[0] & (0x2))
				self.buttonY = (self.message[0] & (0x1))

				self.actions_msg.data = 0 
				if self.buttonA == 2:
					self.actions_msg.data = self.actions_msg.data | 2
					self.get_logger().info("[A] Land sent!")
				if self.buttonB == 4:
					self.get_logger().info("[B]")
					self.actions_msg.data = self.actions_msg.data | 4
				if self.buttonX == 8:
					self.get_logger().info("[X]")
				if self.buttonY == 1:
					self.actions_msg.data = self.actions_msg.data | 1
					self.get_logger().info("[Y] Takeoff sent!")

				if self.actions_msg.data != 0:
					self.actions_pub.publish(self.actions_msg)

				if(self.message[9] & 0x80):
					self.message[1] = 0x55
				if(self.message[9] & 0x40):
					self.message[2] = 0x55
				if(self.message[9] & 0x20):
					self.message[3] = 0x55
				if(self.message[9] & 0x10):
					self.message[4] = 0x55
				if(self.message[9] & 0x08):
					self.message[5] = 0x55
				if(self.message[9] & 0x04):
					self.message[6] = 0x55
				if(self.message[9] & 0x02):
					self.message[7] = 0x55
				if(self.message[9] & 0x01):
					self.message[8] = 0x55


				if self.mode == 0:
					self.subtract = 0
					if(self.message[1] & 0x80):
						self.subtract = 32768
					self.rx = ((self.message[1] & 0x7F) << 8) + self.message[2] - self.subtract
					self.rx = self.rx / 32768
					
					self.subtract = 0
					if(self.message[3] & 0x80):
						self.subtract = 32768
					self.ry = ((self.message[3] & 0x7F) << 8) + self.message[4] - self.subtract
					self.ry = self.ry / 32768

					self.subtract = 0
					if(self.message[5] & 0x80):
						self.subtract = 32768
					self.lx = ((self.message[5] & 0x7F) << 8) + self.message[6] - self.subtract
					self.lx = self.lx / 32768

					self.subtract = 0
					if(self.message[7] & 0x80):
						self.subtract = 32768
					self.ly = ((self.message[7] & 0x7F) << 8) + self.message[8] - self.subtract
					self.ly = self.ly / 32768

					self.stick1 = (self.lx, self.ly)
					self.stick2 = (self.rx, self.ry)
					
					self.mutex.acquire()
					if self.m3_vs_m2 == 1:
						self.x_val   =   self.stick1[1]
						self.y_val   = - self.stick1[0]
						self.z_val   =   self.stick2[1]
						self.yaw_val =   self.stick2[0]
					else:
						self.x_val   =   self.stick2[1]
						self.y_val   = - self.stick2[0]
						self.z_val   =   self.stick1[1]
						self.yaw_val =   self.stick1[0]
					self.mutex.release()

					print(f"x: {self.x_val} \t y: {self.y_val}")
					print(f"z: {self.z_val} \t yaw: {self.yaw_val}")
					print()
				else:
					if self.message[1] & 0x80:
						self.subtract = 2147483648
					else: 
						self.subtract = 0
					self.latitude = ((self.message[1] & 0x7F) << 24) + (self.message[2] << 16) + (self.message[3] << 8) + (self.message[4]) - self.subtract
					self.latitude = self.latitude / IntToFloat

					
					if self.message[5] & 0x80:
						self.subtract = 2147483648
					else: 
						self.subtract = 0
					self.longitude = ((self.message[5] & 0x7F) << 24) + (self.message[6] << 16) + (self.message[7] << 8) + (self.message[8]) - self.subtract
					self.longitude = self.longitude / (IntToFloat / 2)

					print(self.latitude)
					print(self.longitude)
					print()
					

			
def main(args=None):
	rclpy.init(args=args)

	xbox_control = xbox()

	rclpy.spin(xbox_control)

	xbox_control.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()

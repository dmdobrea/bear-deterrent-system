#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import time
import rclpy

import numpy as np
from math import cos, sin, asin, radians, degrees, sqrt, atan2

from rclpy.node import Node
from rclpy.qos  import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy, QoSLivelinessPolicy

# Subscribers
from px4_msgs.msg import VehicleStatus
from px4_msgs.msg import VehicleAttitude
from px4_msgs.msg import VehicleGlobalPosition

# Publishers
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import TimesyncStatus
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import OffboardControlMode

class OffboardVelocityControl(Node):

	def __init__(self):
		super().__init__('GPS_Offb_Control_node')
		qos_profile = QoSProfile(
			reliability = QoSReliabilityPolicy.BEST_EFFORT,		# Gazebo => QoSReliabilityPolicy.BEST_EFFORT
			durability  = QoSDurabilityPolicy.TRANSIENT_LOCAL,      # Gazebo => QoSDurabilityPolicy.TRANSIENT_LOCAL 
			liveliness  = QoSLivelinessPolicy.AUTOMATIC,
			history     = QoSHistoryPolicy.KEEP_LAST,               # Gazebo => QoSHistoryPolicy.KEEP_LAST
			depth       = 1 )

		# internal variables
		self.nav_state    = VehicleStatus.NAVIGATION_STATE_MAX
		self.arming_state = VehicleStatus.ARMING_STATE_ARMED	# Befor (PX4 1.14.3) it worked with ARMING_STATE_MAX, now  (PX4 1.15) does not work anymore
		self.failsafe     = False
		self.flightCheck  = False

		self.local_timestamp = 0
		
		# SUBSCRIBER NODE(S)
		self.vehicle_status_Subscriber = self.create_subscription ( VehicleStatus,   '/fmu/out/vehicle_status_v1', self.vehicle_status_callback, qos_profile )
		self.attitude_Subscriber       = self.create_subscription ( VehicleAttitude, '/fmu/out/vehicle_attitude',  self.attitude_callback,       qos_profile )
		self.gps_Subscriber            = self.create_subscription ( VehicleGlobalPosition, '/fmu/out/vehicle_global_position', self.gps_callback, qos_profile)
		#self.timesync_Subscriber      = self.create_subscription ( TimesyncStatus, '/fmu/out/timesync',          self.timesync_callback,       qos_profile )


		# PUBLISHER NODE(S)
		# for general commands: ARM, DISARM, LAND etc.
		self.vehicle_command_Publisher = self.create_publisher ( VehicleCommand,      "/fmu/in/vehicle_command",       qos_profile )	
		self.offboard_mode_Publisher   = self.create_publisher ( OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile )
		self.trajectory_Publisher      = self.create_publisher ( TrajectorySetpoint,  '/fmu/in/trajectory_setpoint',   qos_profile )

		# timer create
		timer_period = 0.02  # seconds
		self.timer = self.create_timer(timer_period, self.cmd_offboard_timer)

	# GPS
		# GPS reference possition - starting position, for the first time when the system is on
		self.ref_lat = None
		self.ref_lon = None
		
		self.current_lat = None
		self.current_lon = None
		
		# User-defined target
		self.target_lat = 47.213979 # Example destination (replace as needed)
		self.target_lon = 27.524076
		
	# GPS ==> END

                #current yaw value of drone
		self.trueYaw = 0.0
		
		self.current_Ctrl = "GPS"
		
		#initial fly mode!!!!
		self.mode_Possition()

	def gps_callback(self, msg):
		if self.ref_lat is None and msg.lat_lon_valid == True:
			self.ref_lat = msg.lat
			self.ref_lon = msg.lon
			self.get_logger().info(f"GPS reference set: lat={self.ref_lat}, lon={self.ref_lon}")

		if msg.lat_lon_valid == True:
			self.current_lat = msg.lat
			self.current_lon = msg.lon
			# self.get_logger().info(f"GPS current position: lat={self.current_lat}, lon={self.current_lat}")

	#receives current trajectory values from drone and grabs the yaw value of the orientation
	def attitude_callback(self, msg):
		#   +179 N -179
		# +90(V)         -90(E) 
		#   +    S(0)    -
		
		orientation_q = msg.q

		#trueYaw is the drones current yaw value
		self.trueYaw = -(np.arctan2(2.0*(orientation_q[3]*orientation_q[0] + orientation_q[1]*orientation_q[2]),
                             1.0 - 2.0*(orientation_q[0]*orientation_q[0] + orientation_q[1]*orientation_q[1])))
		# self.get_logger().info(f"Current yaw value of drone: {self.trueYaw * 180/3.1415926}")

	def vehicle_status_callback(self, msg):

		if (msg.nav_state != self.nav_state):
                   self.get_logger().info(f"NAV_STATUS: {self.decodeNavState(msg.nav_state)}")

		if (msg.arming_state != self.arming_state):
                   self.get_logger().info(f"ARM STATUS: {msg.arming_state}")   # 1 = disarmed | 2 = armed

		if (msg.failsafe != self.failsafe):
                   self.get_logger().info(f"FAILSAFE: {msg.failsafe}")

		if (msg.pre_flight_checks_pass != self.flightCheck):
                   self.get_logger().info(f"FlightCheck: {msg.pre_flight_checks_pass}")

		self.nav_state    = msg.nav_state
		self.arming_state = msg.arming_state
		self.failsafe     = msg.failsafe
		self.flightCheck  = msg.pre_flight_checks_pass
    
	def decodeNavState (self, navState):
		if (navState == VehicleStatus.NAVIGATION_STATE_AUTO_TAKEOFF):
                   return "AUTO TAKEOFF"
		if (navState == VehicleStatus.NAVIGATION_STATE_POSCTL):
                   return "Position control mode has been set"
		if (navState == VehicleStatus.NAVIGATION_STATE_AUTO_LOITER):
                   return "AUTO LOITER"
		if (navState == VehicleStatus.NAVIGATION_STATE_OFFBOARD):
                   return "OFFBOARD"
		if (navState == VehicleStatus.NAVIGATION_STATE_MANUAL):
                   return "manual mode"
		if (navState == VehicleStatus.NAVIGATION_STATE_ALTCTL):
                   return "altitude control mode"
		if (navState == VehicleStatus.NAVIGATION_STATE_AUTO_MISSION):
                   return "auto mission mode"
		if (navState == VehicleStatus.NAVIGATION_STATE_AUTO_RTL):
                   return "auto return to launch mode"
		if (navState == VehicleStatus.NAVIGATION_STATE_ACRO):
                   return "acro mode"
		if (navState == VehicleStatus.NAVIGATION_STATE_DESCEND):
                   return "descend mode"
		if (navState == VehicleStatus.NAVIGATION_STATE_TERMINATION):
                   return "termination mode"
		if (navState == VehicleStatus.NAVIGATION_STATE_STAB):
                   return "stabilized mode"
		if (navState == VehicleStatus.NAVIGATION_STATE_AUTO_LAND):
                   return "land"
		if (navState == VehicleStatus.NAVIGATION_STATE_AUTO_FOLLOW_TARGET):
                   return "auto follow"
		if (navState == VehicleStatus.NAVIGATION_STATE_ORBIT):
                   return "orbit in a circle"
		else:
                   return navState


	def cmd_offboard_timer(self):
	
		if self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
			if (self.current_Ctrl == "GPS"):

			   # Convert GPS to local NED
				north, east = self.gps_to_local_ned(
					self.ref_lat,    self.ref_lon,
					self.target_lat, self.target_lon
					)
				down = -10.0  # Maintain 10 meters altitude  
				
			    # compute heading between target and starting point 
				heading_angle = self.calculate_heading (
					self.ref_lat,    self.ref_lon,
					self.target_lat, self.target_lon
					)
				heading_angle_degree = (degrees(heading_angle) + 360) % 360	
				# self.get_logger().info(f"Heading angle: {heading_angle_degree:.2f}")
				
			    # set the new trajectory	
				self.newTrajectorySet(north, east, down, heading_angle, True)
				# self.get_logger().info(f"Sending NED setpoint: N={north:.2f}, E={east:.2f}, D={down}")
				
			    # compute the distance between the current UAV position and the GPS target possition	
				dist = self.gps_distance_m(self.target_lat, self.target_lon, self.current_lat, self.current_lon)
				self.get_logger().info(f"Distance to the GPS target possition: {dist}")

				if dist < 1:
					self.get_logger().info("UAV reached the designated location")
					
					self.current_Ctrl = "HOLD"	# set position mode
					
					# go to HOLD (AUTO LOITER) mode!!!!
					self.mode_Hold()
			else:
				self.newTrajectorySet(0.0, 0.0, 0.0, 0.0, False)
		else:
			self.newTrajectorySet(0.0, 0.0, 0.0, 0.0, False)

	# GPS : Convert GPS to local NED		
	def gps_to_local_ned(self, lat_ref, lon_ref, lat_target, lon_target):
		R = 6371000  # meters
		d_lat = radians(lat_target - lat_ref)
		d_lon = radians(lon_target - lon_ref)
		north = d_lat * R
		east = d_lon * R * cos(radians(lat_ref))

		return north, east 
	
	# GPS : Calculate the distance between point A (lat1, lon1) to point B (lat2, lon2)
	def gps_distance_m(self, lat1, lon1, lat2, lon2):
		R = 6371000.0
		phi1    = radians(lat1)
		phi2    = radians(lat2)
		dphi    = radians(lat2 - lat1)
		dlambda = radians(lon2 - lon1)
		a = sin(dphi/2)**2 + cos(phi1)*cos(phi2)*sin(dlambda/2)**2
		
		return 2 * R * asin(sqrt(a))

	# GPS : Calculate the heading (the bearing) from point A (lat1, lon1) to point B (lat2, lon2)
	def calculate_heading(self, lat1, lon1, lat2, lon2):
		# Convert from degrees to radians
		lat1_rad      = radians(lat1)
		lat2_rad      = radians(lat2)
		delta_lon_rad = radians(lon2 - lon1)

		# Compute the components of the formula
		x = sin(delta_lon_rad) * cos(lat2_rad)
		y = cos(lat1_rad) * sin(lat2_rad) - sin(lat1_rad) * cos(lat2_rad) * cos(delta_lon_rad)

		# Compute initial bearing
		initial_bearing = atan2(x, y)

		# Convert from radians to degrees and normalize to 0–359°
		# initial_bearing_deg = (degrees(initial_bearing) + 360) % 360

		# Bearing in degrees from North (0° to 360°)
		return initial_bearing

	
	#=========================================================================================================================
	def newTrajectorySet (self, x_SN, y_VE, z_Down, heading_angle, position = True):
		# Publish offboard control modes
		offboard_msg = OffboardControlMode()
		offboard_msg.timestamp    = int(self.get_clock().now().nanoseconds / 1000)
		
		if position:
			offboard_msg.position     = True
			offboard_msg.velocity     = False
		else:
			offboard_msg.position     = False
			offboard_msg.velocity     = True
			
		offboard_msg.acceleration = False
		offboard_msg.attitude     = False
		offboard_body_rate        = False
		
		self.offboard_mode_Publisher.publish(offboard_msg)	
		#===============================================================
	
		# NED local world frame
		# Publish the trajectory setpoints 
		trajectory_msg = TrajectorySetpoint()
		trajectory_msg.timestamp   = int(self.get_clock().now().nanoseconds / 1000)
		
		if position:
			# X Position in meters (positive is forward or North)
			# Y Position in meters (positive is right or East)
			# Z Position in meters (positive is down)	
			trajectory_msg.position = [x_SN, y_VE, z_Down]
			
			# X velocity in m/s (positive is forward or North)
			# Y velocity in m/s (positive is right or East)
			# Z velocity in m/s (positive is down)
			trajectory_msg.velocity = [float("nan"), float("nan"), float("nan")]
		else:
			# X Position in meters (positive is forward or North)
			# Y Position in meters (positive is right or East)
			# Z Position in meters (positive is down)	
			trajectory_msg.position = [float("nan"), float("nan"), float("nan")]
			
			# X velocity in m/s (positive is forward or North)
			# Y velocity in m/s (positive is right or East)
			# Z velocity in m/s (positive is down)
			trajectory_msg.velocity = [x_SN, y_VE, z_Down]	
		
		# Bearing in degrees from North (0° to 360°)
		trajectory_msg.yaw = heading_angle		# yaw or heading in radians (0 is forward or North)
		
		trajectory_msg.jerk[0] = float("nan")
		trajectory_msg.jerk[1] = float("nan")
		trajectory_msg.jerk[2] = float("nan")		
		trajectory_msg.acceleration[0] = float("nan")	# X acceleration in m/s/s (positive is forward or North)
		trajectory_msg.acceleration[1] = float("nan")	# Y acceleration in m/s/s (positive is right or East)
		trajectory_msg.acceleration[2] = float("nan")	# Z acceleration in m/s/s (positive is down)
		trajectory_msg.yawspeed = 0.0			# yaw rate in rad/s
			
		self.trajectory_Publisher.publish(trajectory_msg)
	
	#=========================================================================================================================	
	def arm(self):
		self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0, 0.0)
		self.get_logger().info("Arm command send")

	def disarm(self):
		self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0, 0.0)
		self.get_logger().info("Disarm command send")

	def takeoff(self):
                # Something new:
                #  param7 is altitude in meters
                #  self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF, param1 = 1.0, param7=5.0)
		self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF)
		self.get_logger().info("Takeoff command send")

	# is working => tested in PX4 v1.16.0
	def mode_Hold(self):
		self.publish_vehicle_command(
			VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 
			1.0, 		# enables custom mode (VEHICLE_MODE_FLAG_CUSTOM_MODE_ENABLED)
			4.0, 		# PX4_CUSTOM_MAIN_MODE_AUTO
			3.0             # PX4_CUSTOM_SUB_MODE_AUTO_LOITER
			) 
		self.get_logger().info("Auto loiter has been send")

	# daca este deja in aer merge foarte bine
	def land(self):
		self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
		self.get_logger().info("Land command send")

	def mode_Possition(self):
		# de verificat: self.publish_vehicle_command(
		#	VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 
		#	1.0, 		# Custom mode enabled
		#	1.0		# PX4_MAIN_MODE_POSCTL
		#	)
		self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 3.0)   #  1.0 => Manual,   2.0 => Altitude,  3.0 => Possition
		self.get_logger().info("Possition mode command send")                            #  4.0 => Mission,  5.0 => Acro,      7.0 => Stabilized,  

	# parameter 1 and 2, as defined by MAVLink uint16 VEHICLE_CMD enum.
	def publish_vehicle_command(self, command, param1=0.0, param2=0.0, param3=0.0, param7=0.0):
		msg = VehicleCommand()
		msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)  # time in microseconds
		msg.param1 = param1
		msg.param2 = param2
		msg.param3 = param3
		msg.param7 = param7           # altitude value in takeoff command
		msg.command = command         # command ID
		msg.target_system     = 1     # system which should execute the command
		msg.target_component  = 1     # component which should execute the command, 0 for all components
		msg.source_system     = 1     # system sending the command
		msg.source_component  = 1     # component sending the command
		msg.from_external     = True
		
		self.vehicle_command_Publisher.publish(msg)

		self.get_logger().info('Next Vehicle Command Set To: "%s"' % msg)
		
def get_topic_list():
	node_dummy = Node("_ros2_dummy_to_show_topic_list")
	time.sleep (0.5)										# it is required AA
	topic_list = node_dummy.get_topic_names_and_types()
	node_dummy.destroy_node()
	return topic_list

def main(args=None):
	rclpy.init(args=args)
	
	#==================================
	no_topis_exist = 0
	
	topics_list = get_topic_list()
	
	#print("\n")
	#print (topics_list)
	#print("\n")
	
	for info in topics_list:
		#print(info[0])
		if info[0] == "/fmu/out/vehicle_status_v1":
			print ("/fmu/out/vehicle_status topic exist!")
			no_topis_exist += 1
		if info[0] == "/fmu/out/timesync_status":
			print ("/fmu/out/timesync_status topic exist!")
			no_topis_exist += 1
		if info[0] == "/fmu/in/vehicle_command":
			print ("/fmu/in/vehicle_command topic exist!")
			no_topis_exist += 1	
		if info[0] == "/fmu/in/trajectory_setpoint":
			print ("/fmu/in/trajectory_setpoint topic exist!")
			no_topis_exist += 1	
		if info[0] == "/fmu/in/offboard_control_mode":
			print ("/fmu/in/offboard_control_mode topic exist !")
			no_topis_exist += 1
		if info[0] == "/fmu/out/vehicle_global_position":
			print ("/fmu/out/vehicle_global_position topic exist !")
			no_topis_exist += 1
	
	if no_topis_exist == 6:
		print ("[INFO] : All required topics exist! We go further ==>\n")
	else:		
		print ("[INFO] : There are missing topics! Exit!\n")
		sys.exit(1)
	#==================================		

	offboard_velocity_control = OffboardVelocityControl()

	rclpy.spin(offboard_velocity_control)

	# Destroy the node explicitly
	# (optional - otherwise it will be done automatically
	# when the garbage collector destroys the node object)
	offboard_velocity_control.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main() 

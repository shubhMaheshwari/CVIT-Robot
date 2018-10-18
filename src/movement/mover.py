#!/usr/bin/env python

# Module to control the robot by manually sending commands to wheel1 and wheel2
# Use this module with care of the robot might have an accident
import time
import sys
import rospy
import json
from std_msgs.msg import String

# Sabertooth connections
from pysabertooth import Sabertooth
import serial.tools.list_ports as port


MAX_SPEED = 30.0
class MoterControl:
	"""
		We have a direct control to the motors using sabertooth
		We send the velocity parameter to both wheel1 and wheel2 
		max vel: 100(No not try this is will break the robot)
		min vel: 0
		For safety the velocity is set to 30.0
	"""

	def __init__(self):
		last_wheel1 = 0
		last_wheel2 = 0
		self.ROBOT_MODE = "MANUAL"

		port = self._find_sabertooth_port()
		self.saber = Sabertooth(port, baudrate=115200, address=128, timeout=0.1)

	def _find_sabertooth_port(self):
		"""
			Find the port sabertooth is connected to
			generally - /dev/ACM0
			somtimes  - /dev/ACM1
		"""
		print "\nDetecting sabertooth....\n"
		pl = list(port.comports())
		print pl
		address = ''
		for p in pl:
		  print p
		  if 'Sabertooth' in str(p):
		      address = str(p).split(" ")
		print "\nAddress found @"
		print address[0]
		return address[0]

	def __move_robot(self,wheel1,wheel2):	
		if wheel1  > MAX_SPEED: 
			wheel1 = MAX_SPEED

		if wheel1 < -MAX_SPEED:
			wheel1 = -MAX_SPEED

		if wheel2  > MAX_SPEED: 
			wheel2 = MAX_SPEED

		if wheel2 < -MAX_SPEED:
			wheel2 = -MAX_SPEED

		print("wheel1:", wheel1, "wheel2",wheel2)
		saber.drive(1, wheel1)
		saber.drive(2, wheel2)		

	def callback(self,data):
		global last_wheel1
		global last_wheel2
		details = json.loads(data.data)
		wheel1 = float(details['wheel1'])
		wheel2 = float(details['wheel2'])
		mode = details['mode']	
		
		
		# Incoming node has the same mode as the current robot mode.
		# Else the input will not we utilized
		if mode == self.ROBOT_MODE:
			print(mode, ROBOT_MODE)
			# Due to current system, track returns 0 0 continously while manual returns it only once 
			if wheel1 == 0 and wheel2 == 0  and (mode == 'TRACK' or mode == "RANDOM"):

				if last_wheel1 <= 5 or last_wheel2 <= 5:
					saber.stop()

				else:				
					for i in range(10):
						# print(speed)
						if last_wheel1 > 0.5:
							last_wheel1 = last_wheel1 - 0.5
						elif last_wheel1 < -0.5:
							last_wheel1 = last_wheel1 + 0.5
						else:
							break

						if last_wheel2 > 0.5:
							last_wheel2 = last_wheel2 - 0.5
						elif last_wheel2 < -0.5:
							last_wheel2 = last_wheel2 + 0.5
						else:
							break

						self.__move_robot(last_wheel1,last_wheel2)
			elif wheel1 == 0 and wheel2 == 0 and mode == 'MANUAL':
				
				STOP_SPEED1 = last_wheel1
				STOP_SPEED2 = last_wheel2
				for i in range(500):
					print(STOP_SPEED1,STOP_SPEED2)
					if STOP_SPEED1 > 0.1:
						STOP_SPEED1 = STOP_SPEED1 - 0.1
					elif STOP_SPEED1 < -0.1:
						STOP_SPEED1 = STOP_SPEED1 + 0.1
					else:
						break

					if STOP_SPEED2 > 0.1:
						STOP_SPEED2 = STOP_SPEED2 - 0.1
					elif STOP_SPEED2 < -0.1:
						STOP_SPEED2 = STOP_SPEED2 + 0.1
					else:
						break

					self.__move_robot(last_wheel1,last_wheel2)
				saber.stop()

				last_wheel1 = 0
				last_wheel2 = 0


			else:
				self.__move_robot(last_wheel1,last_wheel2)
				
				# Gets the last speed which was greater than 0
				last_wheel1 = wheel1
				last_wheel2 = wheel2

	def mode_callback(self.data):
		print(data.data)
		self.ROBOT_MODE = data.data


if __name__ == '__main__':

	mover = MoterControl()
	# spin() simply keeps python from exiting until this node is stopped
	rospy.spin()

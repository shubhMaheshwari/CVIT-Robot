#!/usr/bin/env python
import time, sys
import rospy
import json
from std_msgs.msg import String

import time

from pysabertooth import Sabertooth
import serial.tools.list_ports as port


print "\n Initialization complete"

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

last_wheel1 = 0
last_wheel2 = 0
ROBOT_MODE = "MANUAL"
MAX_SPEED = 30.0
saber = Sabertooth(address[0], baudrate=115200, address=128, timeout=0.1)
def callback(data):
	global last_wheel1
	global last_wheel2
	rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
	details = json.loads(data.data)
	wheel1 = float(details['wheel1'])
	wheel2 = float(details['wheel2'])
	mode = details['mode']	
	print("wheel1:", wheel1, "wheel2",wheel2)
	print("last_wheel1:", last_wheel1, "last_wheel2",last_wheel2)
	
	if wheel1  > MAX_SPEED: 
		wheel1 = MAX_SPEED

	if wheel1 < -MAX_SPEED:
		wheel1 = -MAX_SPEED

	if wheel2  > MAX_SPEED: 
		wheel2 = MAX_SPEED

	if wheel2 < -MAX_SPEED:
		wheel2 = -MAX_SPEED

	if mode == ROBOT_MODE:
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

					saber.drive(1, last_wheel1)
					saber.drive(2, last_wheel2)	

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

				saber.drive(1, STOP_SPEED1)
				saber.drive(2, STOP_SPEED2)	
			saber.stop()

			last_wheel1 = 0
			last_wheel2 = 0


		else:
			saber.drive(1,wheel2)
			saber.drive(2,wheel1)
			
			# Gets the last speed which was greater than 0
			last_wheel1 = wheel1
			last_wheel2 = wheel2

def mode_callback(data):
	global ROBOT_MODE
	# Get the bot mode from the app
	# Currently 2 modes, TRACK and MANUAL
	print(data.data)
	ROBOT_MODE = data.data

def listener():

	# In ROS, nodes are uniquely named. If two nodes with the same
	# node are launched, the previous one is kicked off. The
	# anonymous=True flag means that rospy will choose a unique
	# name for our 'listener' node so that multiple listeners can
	# run simultaneously.
	rospy.init_node('listener', anonymous=True)

	rospy.Subscriber("/movement", String, callback)
	rospy.Subscriber("/robot_mode", String, mode_callback)

	# spin() simply keeps python from exiting until this node is stopped
	rospy.spin()

if __name__ == '__main__':
	listener()
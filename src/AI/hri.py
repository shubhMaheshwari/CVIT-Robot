#!/usr/bin/env pythons
import rospy
import json
import os
from std_msgs.msg import String
from datetime import datetime
import time
import inflect
import numpy as np

MIN_DEPTH_Z  = 400
MAX_DEPTH_Z  = 3500

OBSTACLE_DEPTH = 1600

MIN_X  = 300
MAX_X  = 700
MID_X  = 480

MAX_SPEED = 20

MAX_ROTATE_SPEED = 15

MIN_AREA = 0.001
MID_AREA = 0.05
MAX_AREA = 0.16


fmt = "%Y-%m-%d %H:%M:%S"
p = inflect.engine()
names = {}
DELETE_TIME = 25
VALID_COUNT = 10

# People detection global variables
person_x = 480
person_area = 0 
person_name = '' 
ROBOT_MODE = "RANDOM" 



pub = rospy.Publisher('/speaker', String, queue_size=1)
moter = rospy.Publisher('/movement', String, queue_size=10)
def people_callback(data):

	global person_area
	global person_name
	global person_x
	global person_name
	import time
	obj = json.loads(data.data)

	person_id = np.argmin([ np.fabs(d['x'] - person_x)  for d in obj])
	person_x = int(obj[person_id]['x'])
	person_area = obj[person_id]['area']
	person_name = obj[person_id]['name']

obj_depth = 0
obj_center_depth = 0
obj_x = 0
def object_callback(data):
	global obj_x
	global obj_depth
	global obj_center_depth

	obj_depth_matrix = json.loads(data.data)
	obj_depth_list = np.min(np.array(obj_depth_matrix),axis=0)
	
	obj_x = np.argmax(obj_depth_list)	
	obj_depth = obj_depth_list[obj_x]
	obj_center_depth = np.min(obj_depth_list[12:20])

	obj_x = obj_x*30

def robot_mode_callback(data):
	global ROBOT_MODE
	# Get the bot mode from the app
	# Currently 2 modes, TRACK and MANUAL
	print(data.data)
	ROBOT_MODE = data.data

def listener():
	global time
	global names
	global person_name
	rospy.init_node('listener', anonymous=True)

	rospy.Subscriber("/people_detection", String, people_callback)
	rospy.Subscriber("/object_detection", String, object_callback)
	rospy.Subscriber("/robot_mode", String, robot_mode_callback)

	person_prev_area = person_area
	previous_move = ''
	no_person_count = 0 

	say_things_list = ["Hello", "Hi", "Good Afternoon"]
	rate = rospy.Rate(5) # 10hz
	# Using people_detection and object detection choose the right movement
	while not rospy.is_shutdown():

		print(obj_center_depth, person_area, obj_depth, obj_x+15)

		# Obstacle avoidance
		# Object too close 
		if obj_center_depth < OBSTACLE_DEPTH and (ROBOT_MODE == "RANDOM" or ROBOT_MODE == "TRACK"):

			# Person detected stop 
			if  person_area > MAX_AREA:
				print("Person detected stopping", person_area)
				moter.publish(json.dumps({'mode':ROBOT_MODE,'wheel1':0.0 , 'wheel2': 0.0}))
				previous_move = ' '

				if no_person_count % 40 == 5:
						pub.publish("Welcome to triple I T.")
				no_person_count += 1
				print("No:",no_person_count)
				if person_name not in names:
					names[person_name] = {}
					names[person_name]['count']=1
					names[person_name]['level'] = 0
					names[person_name]['identification_time'] = ''
					

				else:
					names[person_name]['count']+=1
					if names[person_name]['count'] == VALID_COUNT:#(10*names[person]['level'] + 1):
						names[person_name]['level']+=1
						names[person_name]['count'] = 0

				# print("Person:",person_name,"count:",names[person_name]['count'],"level",names[person_name]['level'])
					
				if names[person_name]['level'] >= 1 and  person_name not in ["Unknown", "unknown", "No_person"]:
					identification_time = datetime.strptime(time.strftime("%Y-%m-%d %H:%M:%S", time.gmtime()),fmt)
					if names[person_name]['identification_time'] == '':
						names[person_name]['identification_time'] = identification_time
					else:
						print("Say my name")
						if (identification_time - names[person_name]['identification_time']).total_seconds() > 90:
							names[person_name]['level'] = names[person_name]['count'] = 0
						elif (names[person_name]['level'] == 1 and names[person_name]['count'] == 0):
							names[person_name]['count'] = 1
							names[person_name]['identification_time'] = identification_time
							pub.publish("Hi " + person_name)
						elif (names[person_name]['level'] == 2 and names[person_name]['count'] == 0):
							names[person_name]['count'] = 1
							pub.publish("Hello " + person_name)
							pub.publish("What are you upto these days?")
						elif (names[person_name]['level'] == 3 and names[person_name]['count'] == 0):
							names[person_name]['count'] = 1
							pub.publish("It was nice to talk to you, " + person_name +  " now I must continue to walk ")


			# No possible way out them stop
			elif obj_depth < 1400:
				print("Stopping back ", obj_depth)
				moter.publish(json.dumps({'mode':ROBOT_MODE,'wheel1':0.0 , 'wheel2': 0.0}))
				previous_move = ' '
			else:
				# Else move to the longest path(highest depth)
				if obj_x  < MID_X or previous_move == 'a':
					print("Rotating:a")
					previous_move = 'a'
					moter.publish(json.dumps({'mode':ROBOT_MODE,'wheel1':-MAX_ROTATE_SPEED , 'wheel2': MAX_ROTATE_SPEED}))	

				# Person on right side
				elif obj_x  > MID_X or previous_move == 'd':
					previous_move = 'd'
					print("Rotating:d")
					moter.publish(json.dumps({'mode':ROBOT_MODE,'wheel1': MAX_ROTATE_SPEED , 'wheel2': -MAX_ROTATE_SPEED}))	

				else:
					print("BIZZZAARE")
					moter.publish(json.dumps({'mode':ROBOT_MODE,'wheel1': MAX_ROTATE_SPEED , 'wheel2': -MAX_ROTATE_SPEED}))						
		# Object not too close
		else:
			previous_move = ''
			# Human tracking using object detection  
			# Person is in range of tracking
			if ROBOT_MODE == "TRACK":
				if  (person_area > MIN_AREA and person_area < MAX_AREA):

					if person_x  < MID_X:
						print("Moving:a")
						moter.publish(json.dumps({'mode':'TRACK','wheel1':MAX_SPEED*np.power(float(person_x)/480, 0.8) , 'wheel2': MAX_SPEED}))	

					# Person on right side
					else:
						print("Moving:d")
						moter.publish(json.dumps({'mode':'TRACK','wheel1':MAX_SPEED, 'wheel2':MAX_SPEED*np.power((2-(float(person_x)/480)), 0.8)}))	

				else:
					print("Out of depth or area", person_area)
					moter.publish(json.dumps({'mode':'TRACK','wheel1':0.0 , 'wheel2': 0.0}))


			if ROBOT_MODE == "RANDOM":
				if obj_x  < MID_X:
					print("Moving:a")
					moter.publish(json.dumps({'mode':'RANDOM','wheel1':MAX_SPEED*np.power(float(obj_x)/480, 0.8) , 'wheel2': MAX_SPEED}))	

				# Person on right side
				else:
					print("Moving:d")
					moter.publish(json.dumps({'mode':'RANDOM','wheel1':MAX_SPEED, 'wheel2':MAX_SPEED*np.power((2-(float(obj_x)/480)), 0.8)}))	

		rate.sleep()








	# spin() simply keeps python from exiting until this node is stopped
	# rospy.spin()

if __name__ == '__main__':
	listener()

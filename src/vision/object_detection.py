#!/usr/bin/env python
from __future__ import print_function

# OPENCV Dependencies using ROS
import numpy as np
import sys
import time
import os
import json
try:
	sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
	import cv2
	print("[ INFO] OPENCV-VERSION:",format(cv2.__version__))	
	sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages')	
except:
	pass
import cv2

# ROS Dependencies
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError

# Image Processing Dependecies
import argparse
import dlib
import face_recognition
from face_detect import FaceDetection
from face_recognize import face_recognize

# from YOLO.webcam import detect as yolo_detect
from time import gmtime, strftime
import math
import threading
# initialize the list of class labels MobileNet SSD was trained to
# detect, then generate a set of bounding box colors for each class
CLASSES = ["background", "aeroplane", "bicycle", "bird", "boat",
	"bottle", "bus", "car", "cat", "chair", "cow", "diningtable",
	"dog", "horse", "motorbike", "person", "pottedplant", "sheep",
	"sofa", "train", "tvmonitor"]
COLORS = np.random.uniform(0, 255, size=(len(CLASSES), 3))

# load our serialized model from disk
print("[ INFO] loading model for person detection...")

net = cv2.dnn.readNetFromCaffe("../../models/MobileNetSSD_deploy.prototxt.txt", "../../models/MobileNetSSD_deploy.caffemodel")
frame_counter = 0

# Module to run on kinect

depth_image = None
class image_converter:

	def __init__(self):
		self.pub = rospy.Publisher("/people_detection",String, queue_size=10)
		self.obj_pub = rospy.Publisher("/object_detection",String, queue_size=10)

		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber("/kinect2/qhd/image_color",Image,self.callback)
		self.image_depth_sub = rospy.Subscriber("/kinect2/qhd/image_depth_rect",Image,self.depth_callback)
		self.kinect_image = rospy.Publisher("/webcam_image",CompressedImage, queue_size=10)

		self.person_x = 0
		self.person_y = 0

		# Currently dropping 4 out of 5 frames to reduce lag
		self.kinnect_image_cnt = 0
		self.kinnect_image_mod = 2

		self.face_detect = FaceDetection()

	def depth_callback(self,data):

		depth_image = self.bridge.imgmsg_to_cv2(data, "16UC1")	
		depth_image[depth_image == 0 ] = 1600

		# Average pooling with 60,60
		M, N = 540,960
		K = 30
		L = 30

		MK = M // K
		NL = N // L

		avg = depth_image[:MK*K, :NL*L].reshape(MK, K, NL, L).mean(axis=(1, 3))


		# Get the closest object for each og 60 size row

		self.obj_pub.publish(json.dumps(avg.tolist()))

	def callback(self,data):
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

			self.kinnect_image_cnt += 1
			if self.kinnect_image_cnt % self.kinnect_image_mod == 0:
				details = callback_image(cv_image)
			else:
				details = []

			if len(details) > 0:
				# If no one was present take the person with max area

				if self.person_x == None and self.person_y == None :
					person_id  = np.argmax([ d['area'] for d in details])
				# Else take the person closest 
				else:
					person_id = np.argmin([ np.fabs(d['x'] - self.person_x) + np.fabs(d['y'] - self.person_y) for d in details])

				self.person_x  = details[person_id]['x']
				self.person_y  = details[person_id]['y']


		except CvBridgeError as e:
			print(e)

		except Exception as e:
			print(e)	
		# Send data to listener as json
		try:
			if len(details) > 0:
				# rospy.loginfo(str(details))
				self.pub.publish(json.dumps(details))

				
		except CvBridgeError as e:
			print(e)

		# kinnect_image_cnt += 1
		# if kinnect_image_cnt%10 == 0:
		cv_image = cv2.resize(cv_image, (0,0), fx=0.3, fy=0.3)
		msg = CompressedImage()
		msg.header.stamp = rospy.Time.now()
		msg.format = "jpeg"
		msg.data = np.array(cv2.imencode('.jpg', cv_image)[1]).tostring()
		self.kinect_image.publish(msg)


def kinect_run():
	ic = image_converter()
	rospy.init_node('image_converter', anonymous=True)
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")
		cv2.destroyAllWindows()

def callback_image(frame):
	global person_id
	# global frame_counter
	# print(frame_counter)
	# frame_counter+=1
	# if frame_counter%10 != 0:
	# 	return details
	# if frame is None:
	# 	return []

	
	details = []
	if frame is None:
		return []
	(h, w) = frame.shape[:2]
	blob = cv2.dnn.blobFromImage(cv2.resize(frame, (300, 300)), 0.007843, (300, 300), 127.5)

	net.setInput(blob)

	detections = net.forward()

	for i in np.arange(0, detections.shape[2]):
		confidence = detections[0, 0, i, 2]

		if confidence > 0.25:
			idx = int(detections[0, 0, i, 1])
			box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
			(startX, startY, endX, endY) = box.astype("int")

			label = "{}: {:.2f}%".format(CLASSES[idx], confidence * 100)
			cv2.rectangle(frame, (startX, startY), (endX, endY),
				COLORS[idx], 2)
			y = startY - 15 if startY - 15 > 15 else startY + 15
			cv2.putText(frame, label, (startX, y),
				cv2.FONT_HERSHEY_SIMPLEX, 0.5, COLORS[idx], 2)

			if CLASSES[idx] == "person":

				sub_frame = frame[startY:endY, startX:endX, :]
				face_locations = detect_face(sub_frame)[0]
				# print(face_locations)
				area = float((endX - startX)*(endY - startY))/(h*w)
				if face_locations != () and face_locations != []:
					(face_start_x, face_start_y, face_end_x, face_end_y) = face_locations
					if face_end_x + startX > endX:
						continue	
					cv2.rectangle(frame, (face_start_x + startX, face_start_y + startY), (face_end_x + startX, face_end_y + startY),(255,0,0),2)
					name, score = face_recognize(sub_frame, face_locations)
					face_h = face_end_y - face_start_y
					face_w = face_end_x - face_end_y
					y = startY - 15 if startY - 15 > 15 else startY + 15
					start_x = math.floor(face_start_x - (face_w)/4 + startX)
					end_x = math.ceil(face_end_x + (face_w)/4 + startX)
					start_y = math.floor(face_end_y + 3*(face_h)/16 + startY)
					end_y = math.ceil(face_end_x + 5*(face_h)/2 + startY)
					t_start_x = int(start_x) if start_x > startX else startX
					t_start_y = int(start_y) if start_y < endY else endY
					t_end_x = int(end_x) if end_x < endX else endX
					t_end_y = int(end_y) if end_y < endY else endY

					cv2.rectangle(frame, (t_start_x, t_start_y ), (t_end_x, t_end_y),(255,0,0),2)
					# Area covered by the person of the screen
					# If we detect a face we try to recongnize it else store it as unknown
					details.append({'name': name, 'x':int((startX + endX)/2), 'y':int((startY + endY)/2)  , 'score': score, 'area':area, 'time':strftime("%Y-%m-%d %H:%M:%S", gmtime())})
				else:
					details.append({'name': "unknown", 'x':int((startX + endX)/2), 'y':int((startY + endY)/2) , 'score': 10.0, 'area':area, 'time':strftime("%Y-%m-%d %H:%M:%S", gmtime())})

				# Color detected person
				sub_frame[:,:,2] = sub_frame[:,:,2] + 80
				cv2.circle(frame,((startX + endX)/2, (startY + endY)/2), 3, (0, 255, 0), -1)

	# For no person detected
	if len(details) == 0:
		# print("No person presnet") 
		details.append({'name': "No_person", 'x':int(480), 'y':int(270) , 'score': 10.0, 'area':0.0, 'time':strftime("%Y-%m-%d %H:%M:%S", gmtime())})

	# cv2.imwrite('x.jpg', frame)	
	cv2.imshow('frame', frame)
	if cv2.waitKey(1) & 0xFF == ord('q'):
		sys.exit(0)
	# show the output frame
	return details

def test_run():

	files = ['Night','Afternoon', 'Morning','Rain', 'Distance']
	for file in files:
		print(file)
		for filename in os.listdir('../videos/Face_detect_CVIT_Videos/' + file):
			print(filename)
			video_capture = cv2.VideoCapture(os.path.join('../videos/Face_detect_CVIT_Videos/' + file, filename))
			while(True):
				ret, frame = video_capture.read()
				if frame is None:
					break
				details = callback_image(frame)
				if len(details) > 0:
					print(details)
				if cv2.waitKey(1) & 0xFF == ord('q'):
					break

def webcam_run():
	video_capture = cv2.VideoCapture(0)
	rospy.init_node('image_converter', anonymous=True)
	ros_publisher = rospy.Publisher("/people_detection",String, queue_size=10)
	webcam_publisher = rospy.Publisher("/webcam_image",CompressedImage, queue_size=10)

	cnt = 1;
	while(True):
		ret, frame = video_capture.read()
		details = callback_image(frame)
		if len(details) > 0:
			rospy.loginfo(details)
			ros_publisher.publish(json.dumps(details))



		cnt += 1
		if cnt%2 == 0:
			# frame  = cv2.resize(frame, (0,0), fx=0.3, fy=0.3)
			msg = CompressedImage()
			msg.header.stamp = rospy.Time.now()
			msg.format = "jpeg"
			msg.data = np.array(cv2.imencode('.jpg', frame)[1]).tostring()
			webcam_publisher.publish(msg)



if __name__ == '__main__':
	# frame_counter = 0
	webcam_run()
	# kinect_run()


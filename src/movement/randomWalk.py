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

# from YOLO.webcam import detect as yolo_detect
from time import gmtime, strftime
import math
import threading

class random_walk:

	def __init__(self):
		self.obj_pub = rospy.Publisher("/object_detection",String, queue_size=10)

		self.bridge = CvBridge()
		self.image_depth_sub = rospy.Subscriber("/kinect2/qhd/image_depth_rect",Image,self.depth_callback)
		self.detector = cv2.SimpleBlobDetector_create()

	def depth_callback(self,data):

		depth_image = self.bridge.imgmsg_to_cv2(data, "16UC1")	
			
		# Average pooling with 60,60
		M, N = 540,960
		K = 60
		L = 60

		MK = M // K
		NL = N // L
		depth_image[depth_image == 0 ] = 1600
		avg = depth_image[:MK*K, :NL*L].reshape(MK, K, NL, L).mean(axis=(1, 3))
		

		# Get the closest object for each og 60 size row
		min_region = np.min(avg,axis=0)
		print(min_region)

		depth_image = cv2.line(depth_image, (300,0), (300,540), (0,0,120),4)
		depth_image = cv2.line(depth_image, (480,0), (480,540), (0,0,120),4)
		depth_image = cv2.line(depth_image, (660,0), (660,540), (0,0,120),4)
		depth_image = depth_image*256
		cv2.imshow('frame', depth_image)
		if cv2.waitKey(1) & 0xFF == ord('q'):
			sys.exit(0)	


ic = random_walk()
rospy.init_node('image_converter', anonymous=True)
try:
	rospy.spin()
except KeyboardInterrupt:
	print("Shutting down")
	cv2.destroyAllWindows()
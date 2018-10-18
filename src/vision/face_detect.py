# This file contains the face detection class 
# We can use multiple ways to detect faces from less efficient and faster methods to slower and more efficient ones
# We provide 
# 1. SSD (Single Shot Detection) => Todo
# 2. Dlib's Inbuilt detection => Todo
# 3. Haar Cascade => Todo
# By default we are using SSD 

# Necassary Imports
import numpy as np
import sys
import dlib
import os
from imutils import face_utils

# Due to rospy we are unable to load cv2 directly, hence a hack
try:
	sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
	import cv2
	print("[ INFO] OPENCV-VERSION:",format(cv2.__version__))	
	sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages')	
except Exception as e:
	print(e)
	import cv2
	pass


class FaceDetection:
	"""
	Our face detection class
	Input: Image(numpy array)
	Ouput: List of posible face locations (face_startX , face_startY , face_endX , face_endY)
	"""
	def __init__(self,method='SSD'):
		"""
		param: method -- String which denotes the type of classifier used (default SSD)
		"""

		print("[ INFO] loading model for face detection...")
		if method == "dlib":
			self.method = FaceDetetionDlib()
			self.detect =  self.method.detect_face_dlib
		elif method == "haar":
			self.method = FaceDetetionHaar()
			self.detect =  self.method.detect_face_haar
		else :
			self.method = FaceDetetionSSD()
			self.detect =  self.method.detect_face_SSD

class FaceDetetionSSD:
	"""
	SSD face detection class
	"""
	def __init__(self):
		"""
		For running SSD in opencv we require to import the model and its architecture. We are using res10_300x300 for face detecion
		"""
		self.net = cv2.dnn.readNetFromCaffe("../../models/face_detect.prototxt.txt", "../../models/res10_300x300_ssd_iter_140000.caffemodel")
		self.confidence = 0.4
	def detect_face_SSD(self,frame):
		"""
		Function called by our class
		param: frame -- Image(numpy array)
		return: list of face locations
		"""
		# Check for valid image 
		(h, w) = frame.shape[:2]
		if h == 0 or w == 0:
			return [] 
		# Try to to resize the image to 300x300
		try:
			blob = cv2.dnn.blobFromImage(cv2.resize(frame, (300, 300)), 1.0,
				(300, 300), (104.0, 177.0, 123.0))
		except:
			return []

		# Pass the image as input
		self.net.setInput(blob)
		# Get the ouput from forward prop
		detections = self.net.forward()
		detections = detections[0,0,:,:]
		detections = detections[detections[:,2] > self.confidence]
		print("Hello")
	
		# Return the list of faces located as a 4-tuple
		face_locations = []
		for f in detections:
			(face_startX, face_startY, face_endX, face_endY)  = (f[3:7]*np.array([w,h,w,h])).astype(int)
			face_locations.append((face_startX , face_startY , face_endX , face_endY))

		return face_locations


class FaceDetetionHaar:
	def __init__(self):
		"""
			For running Haar cascade in opencv we require to import the xml file.
		"""
		self.face_cascade = cv2.CascadeClassifier('../../models/haarcascade_frontalface_default.xml')
	
	def detect_face_haar(self,frame):
		"""
		Function called by our class
		param: frame -- Image(numpy array)
		return: list of face locations
		"""
		gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
		face_locations = self.face_cascade.detectMultiScale(gray, 1.3, 5)
		
		return face_locations

class FaceDetetionDlib:
	"""
	We can run inbuild face detection using dlib
	"""

	def __init__(self):
		self.detector = dlib.get_frontal_face_detector()			
	def detect_face_dlib(self,frame):
		"""
		Function called by our class
		param: frame -- Image(numpy array)
		return: list of face locations
		"""
		
		(h, w) = frame.shape[:2]
		if h == 0 or w == 0:
			return [] 

		# Convert to grayscale and run haar cascade
		gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
		rects = self.detector(gray, 1)

		face_locations = []

		for rect in rects:
			(x, y, wid, hei) = face_utils.rect_to_bb(rect)
			face_location = (x , y , x + wid , y + hei )
			face_locations.append(face_location)

		return face_locations
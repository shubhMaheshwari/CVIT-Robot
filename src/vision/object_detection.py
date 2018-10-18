#!/usr/bin/env python
# Using Simple object detection to find different objects in the surrounding of the robot
from __future__ import print_function

# Dependencies
import numpy as np
import sys
import os
import json

try:
	sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
	import cv2
	print("[ INFO] OPENCV-VERSION:",format(cv2.__version__))	
	sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages')	
except:
	import cv2

CLASSES = ["background", "aeroplane", "bicycle", "bird", "boat",
"bottle", "bus", "car", "cat", "chair", "cow", "diningtable",
"dog", "horse", "motorbike", "person", "pottedplant", "sheep",
"sofa", "train", "tvmonitor"]
COLORS = np.random.uniform(0, 255, size=(len(CLASSES), 3))

class ObjectDetection:
	"""
		Our object detecting module. It receives image as input,
		does object detection on it and returns the detections
	"""
	def __init__(self):
		"""
			Iniliazite the class with our CNN(SSD)
		"""

		print("[ INFO] loading model for person detection...")
		# initialize the list of class labels MobileNet SSD was trained to detect
		self.net = cv2.dnn.readNetFromCaffe("../../models/MobileNetSSD_deploy.prototxt.txt", "../../models/MobileNetSSD_deploy.caffemodel")

	def detect(self,frame):
		"""
		:param -- frame numpy array
		:return -- list of objects detected
		"""

		# Assertions
		if frame is None:
			return []
		(h, w) = frame.shape[:2]
		blob = cv2.dnn.blobFromImage(cv2.resize(frame, (300, 300)), 0.007843, (300, 300), 127.5)

		self.net.setInput(blob)

		# Get the objects in the image
		detections = self.net.forward()
		detections = detections[0][0]

		# For all objects with good accuracy show their bounding box
		for i in range(detections.shape[0]):
			confidence = detections[i, 2]

			# If confidence if high draw the boundaries
			if confidence > 0.25:
				idx = int(detections[i, 1])
				box = detections[i, 3:7] * np.array([w, h, w, h])
				(startX, startY, endX, endY) = box.astype("int")

				label = "{}: {:.2f}%".format(CLASSES[idx], confidence * 100)
				cv2.rectangle(frame, (startX, startY), (endX, endY),
					COLORS[idx], 2)
				y = startY - 15 if startY - 15 > 15 else startY + 15
				cv2.putText(frame, label, (startX, y),
					cv2.FONT_HERSHEY_SIMPLEX, 1, COLORS[idx], 1)


		return detections

# Using Object detection and face recongnition this module takes an image and returns the location of the humans in it.

# Necessary imports
import numpy as np
import sys
from time import gmtime, strftime

# Import cv2
try:
	sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
	import cv2
	print("[ INFO] OPENCV-VERSION:",format(cv2.__version__))	
	sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages')	
except:
	import cv2


from object_detection import ObjectDetection,CLASSES
from face_detect import FaceDetection
from face_recognize import FaceRecognition

class PeopleDetection:
	"""
		The class recognizes people from the image and using face recognition detects their name
	"""
	def __init__(self):
		self.obj_detect = ObjectDetection()
		self.detect_face = FaceDetection()
		self.reconize_face = FaceRecognition()

	def detect(self,frame):
		"""
			Detect people
			:param frame -- numpy image
			:return -- details of each person(name, x, y, confidence, time_when_detected)
		"""

		details = []		
		detections = self.obj_detect.detect(frame)

		# Get the objects detected by ObjectDetection
		for i in range(detections.shape[0]):
			confidence = detections[i,2]
			idx = int(detections[i, 1])

			# If the object is a person and high confidence continue
			if confidence > 0.25 and CLASSES[idx] == "person":
					
				(h, w) = frame.shape[:2]		
				box = detections[i, 3:7] * np.array([w, h, w, h])
				(startX, startY, endX, endY) = box.astype("int")

				# In the person subframe detect faces(mostly should get 1 face)
				sub_frame = frame[startY:endY, startX:endX, :]
				face_locations = self.detect_face.detect(sub_frame)
				area = float((endX - startX)*(endY - startY))/(h*w)
				
				# For each face get the person's name
				for face_location in face_locations:
					if face_location == ():
						continue 

					(face_start_x, face_start_y, face_end_x, face_end_y) = face_location
					if face_end_x + startX > endX:
						continue	
					
					# Draw the face
					cv2.rectangle(frame, (face_start_x + startX, face_start_y + startY), (face_end_x + startX, face_end_y + startY),(255,0,0),2)
					
					# Using Face recognition get the face and its score
					name, score = self.reconize_face.face_recognize(sub_frame, face_location)
					
					# Important Conditions
					face_h = face_end_y - face_start_y
					face_w = face_end_x - face_end_y
					y = startY - 15 if startY - 15 > 15 else startY + 15
					start_x = np.floor(face_start_x - (face_w)/4 + startX)
					end_x = np.ceil(face_end_x + (face_w)/4 + startX)
					start_y = np.floor(face_end_y + 3*(face_h)/16 + startY)
					end_y = np.ceil(face_end_x + 5*(face_h)/2 + startY)
					t_start_x = int(start_x) if start_x > startX else startX
					t_start_y = int(start_y) if start_y < endY else endY
					t_end_x = int(end_x) if end_x < endX else endX
					t_end_y = int(end_y) if end_y < endY else endY

					cv2.rectangle(frame, (t_start_x, t_start_y ), (t_end_x, t_end_y),(255,0,0),2)
					# Area covered by the person of the screen
					details.append({'name': name, 'x':int((startX + endX)/2), 'y':int((startY + endY)/2)  , 'score': score, 'area':area, 'time':strftime("%Y-%m-%d %H:%M:%S", gmtime())})

				# Color detected person
				sub_frame[:,:,2] = sub_frame[:,:,2] + 80
				cv2.circle(frame,((startX + endX)/2, (startY + endY)/2), 3, (0, 255, 0), -1)

		# For no person detected, we require this for path planning
		if len(details) == 0:
			details.append({'name': "No_person", 'x':int(480), 'y':int(270) , 'score': 10.0, 'area':0.0, 'time':strftime("%Y-%m-%d %H:%M:%S", gmtime())})

		cv2.imshow('frame', frame)
		if cv2.waitKey(1) & 0xFF == ord('q'):
			sys.exit(0)
		return details

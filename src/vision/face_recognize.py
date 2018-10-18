# Face recognition for detecting face of the user 
# We are using google's facenet for extracting features from the face. 
# We output the name which has the most matched features

# Necessary Imports
import dlib
import numpy as np 
import PIL.Image
import json
import cv2
from imutils import face_utils,resize

# For plotting the features and understanding face distribuition
from sklearn.decomposition import PCA
import matplotlib.pyplot as plt 

# Import models for face recongnition
try:
	import face_recognition_models
except Exception:
	print("Please install `face_recognition_models` with this command before using `face_recognition`:\n")
	print("pip install git+https://github.com/ageitgey/face_recognition_models")
	quit()



class FaceRecognition:
	"""
		Our face recognition class
		Input: Image,Face Location in Image
		Output: Face name  
	"""

	def __init__(self,landmark_model="large"):
		"""
			Load previously encoded faces and their features
		"""
		with open('../../data/face_recognize_features.json') as f:
			faces = json.load(f)

		face_list = []
		face_name_list = []
		for face in faces: 
			for encoding in face['encodings']:
				face_list.append(np.array(encoding))
				face_name_list.append(face['name'])

		self.face_list = np.array(face_list)
		self.face_name_list = face_name_list

		if landmark_model == "small":
			predictor_5_point_model = face_recognition_models.pose_predictor_five_point_model_location()
			self.pose_predictor = dlib.shape_predictor(predictor_5_point_model)
		else:
			predictor_68_point_model = face_recognition_models.pose_predictor_model_location()
			self.pose_predictor = dlib.shape_predictor(predictor_68_point_model)
		
		face_recognition_model = face_recognition_models.face_recognition_model_location()
		self.face_encoder = dlib.face_recognition_model_v1(face_recognition_model)


	def _raw_face_landmarks(self,face_image, face_location=None, model="large"):
		"""
			Return the landmarks of face
			param: face_image -- (numpy array)
			param: face_location -- [top, right, bottom, left]
			return: landmark locations
		"""
		
		face_location = dlib.rectangle(face_location[0], face_location[1], face_location[2], face_location[3])
		return self.pose_predictor(face_image, face_location)

	def _face_distance(self,face_encodings):
		"""
		Given a list of face encodings, compare them to a known face encoding and get a euclidean distance
		for each comparison face. The distance tells you how similar the faces are.

		:param:  face_encodings: encoding of the new_image
		:return: A numpy ndarray with the distance for each face in the same order as the 'faces' array
		"""
		if len(face_encodings) == 0:
			return np.empty((0))

		distance = np.linalg.norm(self.face_list - face_encodings,axis=1)
		idx =  np.argmin(distance)
		min_val = distance[idx]

		if min_val > 0.6:
			return -1,min_val
		else:
			return idx,min_val

	def face_recognize(self,face_image, face_locations, num_jitters=1):
		"""
			Our function to recognize faces
			param: face_image
		"""

		# Get the landmarks 
		raw_landmarks = self._raw_face_landmarks(face_image, face_locations, model="large")

		# Draw the landmarks
		landmarks_np = face_utils.shape_to_np(raw_landmarks)
		for mark in landmarks_np:
			x,y = mark
			cv2.circle(face_image,(x, y), 1, (0, 0, 255), -1)

		try:
			cv2.imshow("Face",face_image[face_locations[1]: face_locations[3],face_locations[0]: face_locations[2],: ])
		except Exception as e: 
			print(e)
	 
	 	# Run the FaceNet and get the encoding
		encoding = np.array(self.face_encoder.compute_face_descriptor(face_image, raw_landmarks, 10))

		# Get the id and distance
		face_id,face_confidence = self._face_distance(encoding)

		if face_id == -1:
			return "Unknown", face_confidence
		else:
			return self.face_name_list[face_id], face_confidence

def do_PCA():
	"""
		We want to see the relations between face encoding of different people 
		Hence best way to visualize is using PCA 
	"""

	# Load face encodings
	with open('../../data/face_recognize_features.json') as f:
			faces = json.load(f)

	face_list = []
	face_name_list = []
	for face in faces: 
		for encoding in face['encodings']:
			face_list.append(np.array(encoding))
			face_name_list.append(face['name'])

	face_list = np.array(face_list)
	fig = plt.figure()
	
	# Compute PCA
	pca = PCA(2)
	pca.fit(face_list)
	faces = pca.transform(face_list)
	
	# Display images
	for i,face_point in enumerate(faces):
		plt.scatter(face_point[0] , face_point[1], label=face_name_list[i])
	plt.title('Faces relations')
	plt.legend()
	plt.show()


if __name__ == "__main__":
	do_PCA() 
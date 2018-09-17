from face_detect import detect_face_SSD
import cv2
import dlib
import json
from face_recognize import _raw_face_landmarks
from imutils import face_utils
import numpy as np

try:
	import face_recognition_models
except Exception:
	print("Please install `face_recognition_models` with this command before using `face_recognition`:\n")
	print("pip install git+https://github.com/ageitgey/face_recognition_models")
	quit()
face_recognition_model = face_recognition_models.face_recognition_model_location()

face_encoder = dlib.face_recognition_model_v1(face_recognition_model)
cap = cv2.VideoCapture(0)

typ = {'front': 0, 'right': 0, 'left': 0}
encodings = []
while(True):
	ret, frame = cap.read()
	try:
		(x1, y1, x2, y2) = detect_face_SSD(frame)
		cv2.rectangle(frame,(x1,y1),(x2, y2),(255,0,0),2)
	except Exception as e:
		pass
	font = cv2.FONT_HERSHEY_SIMPLEX
	flag=0 if typ['front'] == 3 else 1
	if flag is 1:
		if typ['front'] < 3:
			cv2.putText(frame,'Look towards the camera(' + str(typ['front']) + '/3)',(30,50), font, 1,(0,0,255),2)
		# elif typ['right'] < 2:
		# 	cv2.putText(frame,'Turn right(' + str(typ['right']) + '/2)',(30,50), font, 1,(0,0,255),2)
		# elif typ['left'] < 2:
		# 	cv2.putText(frame,'Turn left(' + str(typ['left']) + '/2)',(30,50), font, 1,(0,0,255),2)
		cv2.putText(frame,'Press Key \'s\' When Ready',(30,70), font, 0.5,(0,0,0),1	)
	else:
		cv2.destroyAllWindows()
		break


	cv2.imshow('frame',frame)
	key = cv2.waitKey(1) & 0xFF

	if key == ord('s'):
		print('received input')
		if typ['front'] < 3:
			typ['front']+=1
		# elif typ['right'] < 2:
		# 	typ['right']+=1
		# elif typ['left'] < 2:
		# 	typ['left']+=1
		raw_landmarks = _raw_face_landmarks(frame, (x1, y1, x2, y2), model="large")
		landmarks_np = face_utils.shape_to_np(raw_landmarks)
		encoding = np.array(face_encoder.compute_face_descriptor(frame, raw_landmarks, 10))
		encodings.append(list(encoding))

	
	if key == ord('q'):
		cv2.destroyAllWindows()
		quit()

print("THANKS")
print("Please Enter Your Name")
with open('faces_new.json') as f:
	faces = json.load(f)
Name = raw_input()
member = {}
member['name'] = Name
member['encodings'] = encodings
member['id'] = len(faces)

faces.append(member)
with open('faces_new.json','w') as f:
	json.dump(faces, f, indent = 4)

# print(member)

import face_recognition
import sys
import cv2

print("Loading Image")
image = face_recognition.load_image_file(sys.argv[1])
print("Creating encoding")
face_encoding = face_recognition.face_encodings(image)[0]
# print(",".join(list(face_encoding)))
print(repr(face_encoding))
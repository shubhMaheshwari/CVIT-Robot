# Vision Module is responsible for all the image processing. It has 2 modules.
# Kinect Module: Connect to the kinect using this module and passes the results for path planning
# Webcam: Uses the laptops webcam for video data and passes it for path planning

# Neseccary imports
import json
import sys
import numpy as np

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

from people_detection import PeopleDetection

class KinectModule:
	"""
		Python module which connects all the to the kinect using ros. It uses object detection to detect people.
		Then using face recognition it finds the person and return 
	"""
	def __init__(self):
		"""
			Initialize the kinect module. It starts all the publishers and subscribers
		"""
		self.people_pub = rospy.Publisher("/people_detection",String, queue_size=10)
		self.processed_image = rospy.Publisher("/processed_image",CompressedImage, queue_size=10)

		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber("/kinect2/qhd/image_color",Image,self.callback)
		self.image_depth_sub = rospy.Subscriber("/kinect2/qhd/image_depth_rect",Image,self.depth_callback)

		self.person_x = 0
		self.person_y = 0
		self.person_id = -1

		self.people_detect = PeopleDetection()

	def callback(self,data):
		"""
			Callback function for the kinect
			It takes the input image from the image and uses people detection module to get the details about the user
			Then passes it on for path planning
			:param data -- brg8 data send from kinect
		"""
		try:
			# Load the image
			cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

			# Get locations from people detection module
			details = self.people_detect.detect(cv_image)

			if len(details) > 0:

				# If no one was present take the person with max area
				if self.person_x == None and self.person_y == None :
					self.person_id  = np.argmax([ d['area'] for d in details])
				# Else take the person closest to the last known location
				else:
					self.person_id = np.argmin([ np.fabs(d['x'] - self.person_x) + np.fabs(d['y'] - self.person_y) for d in details])

				# Store person details 
				self.person_x  = details[self.person_id]['x']
				self.person_y  = details[self.person_id]['y']
				print(details)
				self.people_pub.publish(json.dumps(details))


			cv_image = cv2.resize(cv_image, (0,0), fx=0.3, fy=0.3)
			msg = CompressedImage()
			msg.header.stamp = rospy.Time.now()
			msg.format = "jpeg"
			msg.data = np.array(cv2.imencode('.jpg', cv_image)[1]).tostring()
			self.processed_image.publish(msg)

		
		except CvBridgeError as e:
			print("CvBridgeError:",e)

		except Exception as e:
			print("Error",e)	
		

	def depth_callback(self,data):
		"""
			Use the depth image from kinect for better path planning
		"""

		depth_image = self.bridge.imgmsg_to_cv2(data, "16UC1")	
		depth_image[depth_image == 0 ] = 1600

		# Average pooling with 60,60
		M, N = 540,960
		K = 30
		L = 30

		MK = M // K
		NL = N // L

		# Get the depth values of 60x60 blocks
		avg = depth_image[:MK*K, :NL*L].reshape(MK, K, NL, L).mean(axis=(1, 3))

		# Get the closest object for each og 60 size row
		self.people_pub.publish(json.dumps(avg.tolist()))


def kinect_run():
	"""
		Run people detection on kinect 
	"""

	ic = KinectModule()
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")
		cv2.destroyAllWindows()

def webcam_run():
	"""
		Run people detection on webcam 
		and publish the data
	"""
	video_capture = cv2.VideoCapture(0)
	ros_publisher = rospy.Publisher("/people_detection",String, queue_size=10)
	processed_image = rospy.Publisher("/processed_image",CompressedImage, queue_size=10)
	people_detect = PeopleDetection()

	while(True):
		# Get image from webcam
		ret, frame = video_capture.read()
		details = people_detect.detect(frame)

		if len(details) == 0:
			continue
		
		rospy.loginfo(details)
		ros_publisher.publish(json.dumps(details))


		# frame  = cv2.resize(frame, (0,0), fx=0.3, fy=0.3)
		msg = CompressedImage()
		msg.header.stamp = rospy.Time.now()
		msg.format = "jpeg"
		msg.data = np.array(cv2.imencode('.jpg', frame)[1]).tostring()
		processed_image.publish(msg)



if __name__ == '__main__':
	rospy.init_node('image_processing', anonymous=True)
	# webcam_run()
	kinect_run()


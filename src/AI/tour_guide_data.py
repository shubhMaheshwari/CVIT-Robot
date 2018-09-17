# This is a just a testing file to test if some one is able to listen to things being published 

#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
import json 

with open('./tour_guide_directions.json') as f:
	json_data = json.load(f)
	#print(json_data[2]["content"])

class image_converter:

	def __init__(self):

		self.query_sub = rospy.Subscriber("/tour_guide",String,self.callback)


		self.pub_story = rospy.Publisher("/tour_guide_data",String, queue_size=10)

		self.speak = rospy.Publisher("/speaker",String, queue_size=10)

		# For development only
		self.prev_location = 0


	def callback(self,data):

		details = data.data
		rospy.loginfo("Received")

		details = details.split('$')
		if len(details) >= 1:
			query = details[0]
			print("Query:",query)

		else:
			print("Unable to read the query",details)
			print("Correct way of sending query: QUERY$ID$...$...")

		try:
			if query == "STORY":

				location_id = int(details[1])
				story_duration = details[2]

				result = filter(lambda place: place['id'] == location_id, json_data)
				if len(result) == 1:
					story = result[0]['content' + story_duration].encode("utf-8")
					name = result[0]['name'].encode("utf-8")

				else:
					print("Unable to get the story::Location:",location_id," duration:",story_duration)
					story = ''
					name = ''

				print("Telling story:",story)
				self.speak.publish(json.dumps(story))
				self.pub_story.publish(json.dumps({'story':story,'name': name, 'direction': ''}))
				# rospy.set_param('/tour_guide_dest/cord', {'x':x, 'y':y, 'z':0.0, })
				# rospy.set_param('/tour_guide_dest/pos', {'r': 0.0, 'p':0.0, 'y':pos })
				# rospy.set_param('/tour_guide_dest/points', ' '.join([repr(x), repr(y), repr(float(0.0))]))
				# rospy.set_param('/tour_guide_dest/yaw', ' '.join([repr(float(0.0)), repr(float(0.0)), repr(pos)]))

			elif query == "DIRECTION":

				next_location_id = int(details[1])				

				result = filter(lambda place: place['id'] == next_location_id, json_data)
				if len(result) == 1:
					next_place_name = result[0]['name'].encode("utf-8")
					next_place_direction = result[0]['direction'].encode("utf-8")

				else:
					print("Unable to get the next_location")
					next_place = ''

				print("Next location",next_place_name)
				self.speak.publish(json.dumps("Next we will move to " + next_place_name))
				self.speak.publish(json.dumps(".".join(next_place_direction.split(';'))))
				self.pub_story.publish(json.dumps({'story':'', 'name': next_place_name, 'direction': next_place_direction}))
				# rospy.set_param('/tour_guide_dest/cord', {'x':x, 'y':y, 'z':0.0, })
				# rospy.set_param('/tour_guide_dest/pos', {'r': 0.0, 'p':0.0, 'y':pos })
				# rospy.set_param('/tour_guide_dest/points', ' '.join([repr(x), repr(y), repr(float(0.0))]))
				# rospy.set_param('/tour_guide_dest/yaw', ' '.join([repr(float(0.0)), repr(float(0.0)), repr(pos)]))

		except Exception as e:
			print(e)

def tour_guide_run():
	ic = image_converter()
	rospy.init_node('tour_guide_module', anonymous=True)
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")
		cv2.destroyAllWindows()


if __name__ == '__main__':
	try:
		tour_guide_run()
	except rospy.ROSInterruptException:
		pass
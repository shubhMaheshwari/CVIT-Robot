#!/usr/bin/env python
import rospy
import json
import time
import os
import sys
import time
import threading
import re 

from std_msgs.msg import String

from gtts import gTTS
import alsaaudio
import wave
import audioop

import speech_recognition as sr
import requests
speaking = False

print("Speech Recognition Version:",sr.__version__)

class SpeechToText():

	def __init__(self):

		self.speech_ind = 0
		self.old_offset = 0
		self.CALL_CNT = 2500
		self.FILENAME = 'out.wav'

		# Open file to record sound
		self.file = wave.open(self.FILENAME, 'wb')
		self.file.setparams((2,2,22050,0,'NONE', 'NONE'))

		# Speech to text class
		self.r = sr.Recognizer()

		# Command List
		self.cmd_list = []

	def play(self):
		os.system('aplay -f S16_LE -c 1 ' + self.FILENAME)


	def get_duration(self):
		frames = self.file.getnframes()
		rate = self.file.getframerate()
		duration = frames / float(rate)
		return duration


	def speech_to_text(self):

		# print("Total duration:", self.get_duration())
		# print("Predicted Text")
		speech = sr.AudioFile(self.FILENAME)
		with speech as source:
			audio = self.r.record(source, offset= self.old_offset)
			try:
				cmd = self.r.recognize_google(audio)
				print("Text:",cmd)
				self.cmd_list.append(cmd)
				self.run_new_cmd(cmd)

			except Exception as e:
				print("E:",e)

		self.old_offset = self.get_duration()
		# print("Thread completed")

	def run_new_cmd(self,cmd):

		# If else statements simple command interface 
		if "exit" in cmd:

			with open("log.txt", 'w') as f:
				f.write(str(self.cmd_list))

			# sys.exit(0) will only kill the thread hence we need 
			os._exit(0)
		elif "track" in cmd or "follow" in cmd:
			robot_mode_pub.publish("TRACK")
			speaker_pub.publish("Changing to Tracking mode")

		elif "random walk" in cmd:
			robot_mode_pub.publish("RANDOM")
			speaker_pub.publish("Changing to Random Motion mode")

		elif "stop" in cmd:
			moter_pub.publish(json.dumps({'mode':'TRACK','wheel1':0.0 , 'wheel2': 0.0}))
			moter_pub.publish(json.dumps({'mode':'RANDOM','wheel1':0.0 , 'wheel2': 0.0}))

		elif "joke" in cmd:
			try:
				r = requests.get('https://icanhazdadjoke.com/slack')
				joke = json.loads(r.content)
				joke = joke['attachments'][0]['text']
				speaker_pub.publish(joke)
			except Exception as e:
				print("Couldn't tell the joke:",e)



def callback(data):
	global speaking
	rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
	details = data.data
	speaking = True				
	say(details)
	speaking = False
def say(data):
	global speaking
	try:
		data = data.replace('?','.')
		data = data.split('.')
	except Exception as e:
		print("Expecting string",e)
		return	
	for d in data:
		if d == '' or d == ' ' or d == '\'' or d == '\"':
			continue
		tts = gTTS(d )
		tts.save('audio.mp3')
		os.system('mpg123 audio.mp3')

		

rospy.init_node('listener', anonymous=True)
rospy.Subscriber("/speaker", String, callback)
robot_mode_pub = rospy.Publisher('/robot_mode', String, queue_size=10)
speaker_pub = rospy.Publisher('/speaker', String, queue_size=10)
moter = rospy.Publisher('/movement', String, queue_size=10)


def voice():
	global speaking
	

	# Using people_detection and object detection choose the right movement
	stt = SpeechToText()

		# Open microphone
	device = 'default'
	inp = alsaaudio.PCM(alsaaudio.PCM_CAPTURE, alsaaudio.PCM_NONBLOCK, device=device)
	# Set attributes: Mono, 44100 Hz, 16 bit little endian samples
	inp.setchannels(1)
	inp.setrate(44100)
	inp.setformat(alsaaudio.PCM_FORMAT_S16_LE)

	inp.setperiodsize(160)

	print("Start speaking")
	rate = rospy.Rate(500) # 10hz

	while not rospy.is_shutdown():
		# Read data from device
		# print(speaking)		
		l, data = inp.read()
	  
		if l and not speaking:
			# print("Recording")
			stt.speech_ind += 1
			stt.file.writeframes(data)
			
		if stt.speech_ind % stt.CALL_CNT == 0: 
			threading.Thread(target=stt.speech_to_text).start()
			# print(self.speech_ind)
			# self.speech_to_text()
			# time.sleep(2)
		rate.sleep()
if __name__ == '__main__':
	voice()
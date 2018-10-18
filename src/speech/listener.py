#!/usr/bin/env python
# Speech Module is required for understanding spech from micrphone 
# and converting text to speech 

# We are using googles text2speech and speech2text apis 

# Alot of progress has to be done on this module
# Currently we are using laptop's microphone


# Necessary Imports
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

print("Speech Recognition Version:",sr.__version__)


class SpeechProcessing():
	"""
		Speech processing module for robot
		We are using gTTS(hindi) for text to speech
		And using google's speech recognition for speech to text
		google's speech recognition makes a API request 
		It takes a long time to respond hence it is very slow and inefficient
	"""

	def __init__(self):

		self.is_speaking = False
		self.old_offset = 0

		self.robot_mode_pub = rospy.Publisher('/robot_mode', String, queue_size=10)
		self.moter = rospy.Publisher('/movement', String, queue_size=10)


		rospy.Subscriber("/speaker", String, self.text_to_speech_callback)

		# Open file to record sound
		self.file = wave.open('../../data/out.wav', 'wb')
		self.file.setparams((2,2,22050,0,'NONE', 'NONE'))

		# Speech to text class
		self.r = sr.Recognizer()

		# Command List
		self.cmd_list = []



	def get_duration(self):
		frames = self.file.getnframes()
		rate = self.file.getframerate()
		duration = frames / float(rate)
		return duration


	def speech_to_text(self):

		# print("Total duration:", self.get_duration())
		# print("Predicted Text")
		speech = sr.AudioFile('../../data/out.wav')
		with speech as source:
			audio = self.r.record(source, offset= self.old_offset)
			try:
				cmd = self.r.recognize_google(audio)
				print("Text:",cmd)
				self.run_new_cmd(cmd)

			except Exception as e:
				print("E:",e)

		self.old_offset = self.get_duration()

	def text_to_speech_callback(self):
		rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
		details = data.data
		self.is_speaking = True				
		self.say(details)
		self.is_speaking = False

		

	def say(self,data):
		try:
			data = data.replace('?','.')
			data = data.split('.')
		except Exception as e:
			print("Expecting string",e)
			return	
		for d in data:
			if d == '' or d == ' ' or d == '\'' or d == '\"':
				continue
			tts = gTTS(d)
			tts.save('../../data/audio.mp3')
			os.system('mpg123 ../../data/audio.mp3')


	def start_listening(self):

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
		speech_ind = 0
		CALL_CNT = 1500
		while not rospy.is_shutdown():
			# Read data from device
			# print(self.is_speaking)
			l, data = inp.read()
		  
			if l and not self.is_speaking:
				# print("Recording")
				speech_ind += 1
				self.file.writeframes(data)
				
			if speech_ind % CALL_CNT == 0: 
				threading.Thread(target=self.speech_to_text).start()
			rate.sleep()

	def run_new_cmd(self,cmd):

		# If else statements simple speech command   
		if "exit" in cmd:
			# sys.exit(0) will only kill the thread hence we need 
			os._exit(0)
		elif "track" in cmd or "follow" in cmd:
			self.robot_mode_pub.publish("TRACK")
			self.say("Changing to Tracking mode")

		elif "random walk" in cmd:
			self.robot_mode_pub.publish("RANDOM")
			self.say("Changing to Random Motion mode")

		elif "stop" in cmd:
			self.moter_pub.publish(json.dumps({'mode':'TRACK','wheel1':0.0 , 'wheel2': 0.0}))
			self.moter_pub.publish(json.dumps({'mode':'RANDOM','wheel1':0.0 , 'wheel2': 0.0}))

		elif "joke" in cmd:
			try:
				r = requests.get('https://icanhazdadjoke.com/slack')
				joke = json.loads(r.content.decode('utf-8')	)
				joke = joke['attachments'][0]['text']
				print(joke)
				self.say(joke)
			except Exception as e:
				print("Couldn't tell the joke:",e)


if __name__ == '__main__':

	rospy.init_node('speech_processing', anonymous=True)
	# Initialize and start self.is_speaking module
	stt = SpeechProcessing()
	
	# Start listening from the laptop's microphone
	stt.start_listening()

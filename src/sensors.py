import RPi.GPIO as GPIO
import time
import os
import cv2
import numpy as np
import picamera
import picamera.array

class Sensor:
	"""
	Methods for parsing sensor data
	"""
	
	def __init__(self):
		print("Initializing Sensors...")
		self.TRIG = 16
		self.ECHO = 12
		
		GPIO.setmode(GPIO.BCM)
		GPIO.setup(self.TRIG,GPIO.OUT)
		GPIO.setup(self.ECHO,GPIO.IN)
		
		GPIO.output(self.TRIG,GPIO.LOW)

		self.time_limit = 0.01
		self.rate = 0.0001
		self.prior_distance = 0

		# load the pre-trained face detection model
		self.file_dir = os.path.dirname(os.path.realpath(__file__)) + '/haarcascade_frontalface_default.xml'
		self.face_cascade = cv2.CascadeClassifier(self.file_dir)
		
	def sense_distance(self):
		time_start = time.time()
		time_elapsed = 0.0
		GPIO.output(self.TRIG,GPIO.HIGH)
		time.sleep(self.rate)
		GPIO.output(self.TRIG,GPIO.LOW)
		
		while (GPIO.input(self.ECHO) == 0) & (time_elapsed < self.time_limit):
			pulse_start = time.time()
			time_elapsed = time.time() - time_start
			
		while GPIO.input(self.ECHO) == 1:
			pulse_end = time.time()
			
		try:
			pulse_duration = pulse_end - pulse_start
			distance = pulse_duration * 17150
			self.prior_distance = distance
		except:
			distance = self.prior_distance
			print('using prior distance')
		return distance
	
	def sense_camera(self):
		with picamera.PiCamera() as camera:
			with picamera.array.PiRGBArray(camera) as stream:

				camera.capture(stream, 'bgr', use_video_port=True)
				frame = stream.array
				frame_center = [frame.shape[1]//2, frame.shape[0]//2]
				
				gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

				faces = self.face_cascade.detectMultiScale(gray, scaleFactor=1.2, minNeighbors=5)
				face_vector_norm = []

				if len(faces) > 0:
					idx_max_face = np.argmax(faces[:,2]*faces[:,3])
					(x, y, w, h) = faces[idx_max_face,:]
					face_center = [x + w // 2, y + h // 2]
					face_vector = [face_center[0] - frame_center[0], face_center[1] - frame_center[1]]
					face_vector_norm = [face_vector[0]/max(frame.shape)*2,face_vector[1]/max(frame.shape)*2]

				return face_vector_norm

	def sense_audio(self):
		pass

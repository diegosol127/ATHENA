import RPi.GPIO as GPIO
import time
import os
import cv2
import picamera
import picamera.array
import smbus

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
				camera.resolution = (640, 480)
				frame_center = (camera.resolution[0] // 2, camera.resolution[1] // 2)

				camera.capture(stream, 'bgr', use_video_port=True)
				frame = stream.array

				gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

				faces = self.face_cascade.detectMultiScale(gray, scaleFactor=1.2, minNeighbors=5)
				face_vectors = []

				for (x, y, w, h) in faces:
					face_center = (x + w // 2, y + h // 2)
					face_vector = (face_center[0] - frame_center[0], face_center[1] - frame_center[1])
					face_vectors.append(face_vector)

				return face_vectors

	def sense_audio(self):
		pass

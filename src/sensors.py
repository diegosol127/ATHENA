import RPi.GPIO as GPIO
import time
import cv2

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
		pass

	def sense_audio(self):
		pass

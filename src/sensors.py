import RPi.GPIO as GPIO
import time

class Sensor:
	def __init__(self):
		print("Initializing Sensors...")
		self.TRIG = 16
		self.ECHO = 12
		
		GPIO.setmode(GPIO.BCM)
		GPIO.setup(self.TRIG,GPIO.OUT)
		GPIO.setup(self.ECHO,GPIO.IN)
		
		GPIO.output(self.TRIG,GPIO.LOW)
		
	def sense_distance(self):
		GPIO.output(self.TRIG,GPIO.HIGH)
		time.sleep(0.00001)
		GPIO.output(self.TRIG,GPIO.LOW)
		
		while GPIO.input(self.ECHO) == 0:
			pulse_start = time.time()
			
		while GPIO.input(self.ECHO) == 1:
			pulse_end = time.time()
			
		pulse_duration = pulse_end - pulse_start
		distance = pulse_duration * 17150
		return distance

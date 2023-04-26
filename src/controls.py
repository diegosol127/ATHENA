import RPi.GPIO as GPIO
from time import sleep

class Controller:
	"""
	Methods for controlling the motion of a robot
	"""
	
	def __init__(self):
		print("Initializing Controller...")
		self.IN1 = 27
		self.IN2 = 17
		self.IN3 = 23
		self.IN4 = 24
		self.ENA = 22
		self.ENB = 25
		
		GPIO.setmode(GPIO.BCM)
		GPIO.setup(self.IN1,GPIO.OUT)
		GPIO.setup(self.IN2,GPIO.OUT)
		GPIO.setup(self.IN3,GPIO.OUT)
		GPIO.setup(self.IN4,GPIO.OUT)
		GPIO.setup(self.ENA,GPIO.OUT)
		GPIO.setup(self.ENB,GPIO.OUT)
		
		GPIO.output(self.IN1,GPIO.LOW)
		GPIO.output(self.IN2,GPIO.LOW)
		GPIO.output(self.IN3,GPIO.LOW)
		GPIO.output(self.IN4,GPIO.LOW)
		
		self.pA = GPIO.PWM(self.ENA,30)
		self.pB = GPIO.PWM(self.ENB,30)
		self.dcA = 0
		self.dcB = 0
		self.pA.start(self.dcA)
		self.pB.start(self.dcB)
		
	def move_forward(self,time):
		GPIO.output(self.IN1,GPIO.HIGH)
		GPIO.output(self.IN2,GPIO.LOW)
		GPIO.output(self.IN3,GPIO.HIGH)
		GPIO.output(self.IN4,GPIO.LOW)
		sleep(time)
		
	def move_backward(self,time):
		GPIO.output(self.IN1,GPIO.LOW)
		GPIO.output(self.IN2,GPIO.HIGH)
		GPIO.output(self.IN3,GPIO.LOW)
		GPIO.output(self.IN4,GPIO.HIGH)
		sleep(time)
		
	# verify direction... (my right or your right)
	def move_right(self,time):
		GPIO.output(self.IN1,GPIO.LOW)
		GPIO.output(self.IN2,GPIO.HIGH)
		GPIO.output(self.IN3,GPIO.HIGH)
		GPIO.output(self.IN4,GPIO.LOW)
		sleep(time)
		
	# verify direction... (my left or your left)
	def move_left(self,time):
		GPIO.output(self.IN1,GPIO.HIGH)
		GPIO.output(self.IN2,GPIO.LOW)
		GPIO.output(self.IN3,GPIO.LOW)
		GPIO.output(self.IN4,GPIO.HIGH)
		sleep(time)

	def stop_moving(self,time):
		GPIO.output(self.IN1,GPIO.LOW)
		GPIO.output(self.IN2,GPIO.LOW)
		GPIO.output(self.IN3,GPIO.LOW)
		GPIO.output(self.IN4,GPIO.LOW)
		sleep(time)
	
	def set_speed(self,duty_cycle):
		self.pA.ChangeDutyCycle(duty_cycle)
		self.pB.ChangeDutyCycle(duty_cycle)

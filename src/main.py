
# Python libraries
import threading
from time import sleep
from RPi.GPIO import cleanup
# Local files
import controls
import sensors

class EmergentBehavior:
	def __init__(self):
		print("ATHENA is booting up")
		self.athena_controller = controls.Controller()
		self.athena_sensor = sensors.Sensor()
		self.rate = 0.1
		self.distance = 100.0
		
		self.thread_dist = threading.Thread(target=self.callback,args=())
		self.thread_dist.start()
		
		self.athena_controller.set_speed(75)
		self.behavior_wander()
	
	def callback(self):
		while True:
			self.distance = self.athena_sensor.sense_distance()
			sleep(0.01)
		
	def behavior_wander(self):
		while self.distance > 20.0:
			print(self.distance)
			print("FORWARD")
			self.athena_controller.move_forward(self.rate)
			sleep(self.rate)
		self.athena_controller.stop_moving(1)
		self.behavior_reverse()
		
	def behavior_reverse(self):
		while self.distance <= 20.0:
			print(self.distance)
			print("BACKWARD")
			self.athena_controller.move_backward(self.rate)
			sleep(self.rate)
		self.athena_controller.stop_moving(1)
		self.behavior_wander()
		
def main():
	try:
		behavior = EmergentBehavior()
	except KeyboardInterrupt:
		cleanup()
	
if __name__ == "__main__":
	main()

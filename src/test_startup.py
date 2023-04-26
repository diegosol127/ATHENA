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

		self.athena_controller.set_speed(75)
		self.behavior_test()
    
	def behavior_test(self):
		print('moving forward')
		self.athena_controller.move_forward(2)
		print('stop')
		self.athena_controller.stop_moving(2)
		print('moving backward')
		self.athena_controller.move_backward(2)
		print('stop')
		self.athena_controller.stop_moving(2)
		print('ending program....')
	
def main():
	try:
		behavior = EmergentBehavior()
	except KeyboardInterrupt:
		cleanup()
	
if __name__ == "__main__":
	main()
	cleanup()
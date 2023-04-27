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
		self.face_buffer = [0,0,0,0,0,0,0,0,0,0]
		self.face_detected = False
		self.face_box_threshold = 20
		
		self.thread_distance = threading.Thread(target=self.callback_distance)
		self.thread_distance.start()
		self.thread_camera = threading.Thread(target=self.callback_camera)
		self.thread_camera.start()
		
		self.athena_controller.set_speed(75)
		self.behavior_wander()
	
	def callback_distance(self):
		while True:
			self.distance = self.athena_sensor.sense_distance()
			sleep(0.01)

	def callback_camera(self):
		while True:
			self.face_vec = self.athena_sensor.sense_camera()

			if self.face_vec:
				self.face_buffer.append(1)
			else:
				self.face_buffer.append(0)
			
			if 1 in self.face_buffer:
				self.face_detected = True
			else:
				self.face_detected = False
			sleep(0.2)
		
	def behavior_wander(self):
		print("FORWARD")
		while (self.distance > 20.0) and (not self.face_vec):
			print(self.distance)
			self.athena_controller.move_forward(self.rate)
			sleep(self.rate)
		self.athena_controller.stop_moving(0.25)
		if self.face_vec:
			self.behavior_tracking()
		else:
			self.behavior_reverse()
		
	def behavior_reverse(self):
		print("BACKWARD")
		while self.distance <= 20.0:
			print(self.distance)
			self.athena_controller.move_backward(self.rate)
			sleep(self.rate)
		self.athena_controller.stop_moving(0.25)
		self.behavior_wander()

	def behavior_tracking(self):
		print("TRACKING")
		while (self.face_detected):
			while (abs(self.face_vec[0]) < self.face_box_threshold):
				if self.face_vec[0] < 0:
					self.athena_controller.rotate_right(0.5)
					sleep(0.5)
				else:
					self.athena_controller.rotate_left(0.5)
					sleep(0.5)
			self.athena_controller.move_forward(0.5)
			sleep(0.5)
			
			

		
def main():
	try:
		behavior = EmergentBehavior()
	except KeyboardInterrupt:
		cleanup()
	
if __name__ == "__main__":
	main()

import controls

def test_motor():
	print("Enter a command:")
	athena = controls.Controller()
	dt = 0.1
	
	try:	
		while(True):
			key = input()
			
			if key == "f":
				print("Forward")
				athena.move_forward(dt)
			
			if key == "b":
				print("Backward")
				athena.move_backward(dt)
				
			if key == "1":
				print("Speed 1")
				athena.set_speed(50)
				
			if key == "2":
				print("Speed 2")
				athena.set_speed(75)
				
			if key == "3":
				print("Speed 3")
				athena.set_speed(100)
				
			if key == "e":
				break

	except KeyboardInterrupt:
		pass
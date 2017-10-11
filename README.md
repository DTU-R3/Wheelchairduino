# wheelchairduino
Arduino-based wheelchair controller

1. The program is used to control the electrical wheelchair or other mobile robot with similar configuration.
2. The components used are a Teensy 3.2 board, a IBT-2 motor driver and a HM-10 Bluthtooth module.
3. The micro-controller receive the command in the format 'DK 1 000 1 000'

	First and second digits: header of the command
	
	Third digit: Turning direction of the left wheel, 0-backward, 1-stop, 2-forward
	
	Fourth to Sixth digits: Turning speed of the left wheel, maximum 255.
	
	Seventh digit: Turning direction of the right wheel, 0-backward, 1-stop, 2-forward
	
	Eighth to Tenth digits: Turning speed of the right wheel, maximum 255.
	

# GPIO example using a push button

# import the GPIO and time libraries
import RPi.GPIO as GPIO

# Set the GPIO mode to BCM mode and disable warnings
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Define pin
btnPin = 20
GPIO.setup(btnPin, GPIO.IN, pull_up_down = GPIO.PUD_UP)

# Begin while loop
while True:
	btnVal = GPIO.input(btnPin)

	# If the pin is low, print to terminal
	if (btnVal == false):
		print(‘Button pressed’)

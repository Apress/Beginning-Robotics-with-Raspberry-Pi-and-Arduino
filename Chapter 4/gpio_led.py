# GPIO example blinking LED

# Import the GPIO and time libraries
import RPi.GPIO as GPIO
import time

# Set the GPIO mode to BCM and disable warnings
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Define pins
led = 16

GPIO.setup(led,GPIO.OUT)

# Make sure LED is off
GPIO.output(led,False)

# Begin Loop
while True:

    # Turn LED on
    GPIO.output(led,True)

    # Wait 1 second
    time.sleep(1)

    # Turn LED off
    GPIO.output(led,False)

    # Wait 1 second
    time.sleep(1)

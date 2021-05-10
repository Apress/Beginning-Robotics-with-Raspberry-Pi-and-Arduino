import serial
import time

ser = serial.Serial(‘/dev/ttyAMC0’, 9600)

while 1:
	recSer = ser.readline().decode(‘utf-8’)
	recSer.rstrip()

	distance = int(recSer + ‘0’)/10

	print(“Distance: “ + str(distance) + “cm     “, end = ‘\r’)
	time.sleep(0.5)

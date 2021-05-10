import serial
import time

ser = serial.Serial('/dev/ttyACM0',9600,timeout=1)
a = 1
b = 2
c = 3

while 1:
    valList = [str(a),str(b),str(c)]
    sendStr = ','.join(valList)

    print(sendStr)

    ser.write(sendStr.encode('utf-8'))

    time.sleep(0.1)

    ser.flushInput()
    recStr = ser.readline().decode('utf-8')
    print(recStr)

    a = a+1
    b = b+1
    c = c+1

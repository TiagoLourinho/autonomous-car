import select
import serial
import time
import sys
import numpy as np

data = []

# Open a connection to the serial port
ser = serial.Serial('/dev/cu.usbserial-1440', 9600)

# Read and store data in a loop
i = 0
while True:
    # Check if the user has pressed the Enter key
    if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
        break

    # Check if there is data available on the serial port
    if ser.in_waiting > 0:
        # Read and store the data
        line = ser.readline()
        if i > 1:
            data.append(line)
        i += 1    
        print(line)

# Close the serial port connection
ser.close()

#data = [b'-8 0 992 -3 -2 0\r\n', b'-9 -4 989 -3 -2 0\r\n', b'-12 -3 991 -3 -2 -1\r\n', b'-9 1 991 -2 -2 0\r\n', b'-12 -2 993 -2 -2 0\r\n']

acc, gyro = [], []
for points in data:
    strings = points.split(b' ')
    acc.append(list(map(float, strings[:3])))
    gyro.append(list(map(float, strings[3:])))
    
g = 9.807; # gravitational accelaration on Earth surface
acc = np.array(acc) * 1e-3 * g # [mg] -> [m/s^2]
gyro = np.array(gyro) # [dps]

# Print the stored data
print("Stored data:")
print("Accelerometer:")
print(acc)
print("Gyroscope:")
print(gyro)
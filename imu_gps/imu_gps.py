import glob
import serial
import select
import sys
import numpy as np


# Find all available serial ports
ports = glob.glob('/dev/tty.usbserial-14*')

# Initialize the serial port variable
ser = None

# Try to connect to each port in turn
for port in ports:
    try:
        ser = serial.Serial(port)
        print(f'Successfully connected to {port}')
        break  # Stop searching for a port once we find one that works
    except:
        pass

# If we didn't find any working ports, print an error message
if ser is None:
    print('Error: Could not find any working serial ports')


wait = 0
while True:
    # Check if the user has pressed the Enter key
    if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
        break

    # Check if there is data available on the serial port
    if ser.in_waiting > 0:
        # Read and store the data
        line = ser.readline()
        if line == b'Not available\r\n':
            if wait == 0:
                print("Calibrating IMU and waiting for GPS signal...")
                wait += 1
            else:
                None
        else:
          strings = line.split(b' ')
          data = np.array(list(map(float, strings[:5])))
          a_x, a_y, w_z, lat, lon = data[0], data[1], data[2], data[3], data[4]  
          print(a_x, a_y, w_z, lat, lon)

# Close the serial port connection
ser.close()
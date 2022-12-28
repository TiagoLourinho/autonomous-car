import select
import serial
import sys
import numpy as np
import matplotlib.pyplot as plt

data = []

# Open a connection to the serial port
ser = serial.Serial('/dev/tty.usbserial-1410')

# Read and store data in a loop
i = 0
acc, gyro = [], []
dt = 100e-3
g = 9.807

x = np.zeros(1000)
v_x = np.zeros(1000)
y = np.zeros(1000)
v_y = np.zeros(1000)
theta = np.zeros(1000)

while True:
    # Check if the user has pressed the Enter key
    if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
        break

    # Check if there is data available on the serial port
    if ser.in_waiting > 0:
        # Read and store the data
        line = ser.readline()
        if i == 0:
            print("Calibrating - Please wait for 12 seconds")
        if i > 0:
            strings = line.split(b' ')
            imu_data = np.array(list(map(float, strings[:3])))
            imu_data[0] *= -1e-3 * g # [mg] -> [m/s^2]
            imu_data[1] *= -1e-3 * g # [mg] -> [m/s^2]
            imu_data[2] *= 1 # np.pi / 180 # [dps = º/s] -> [rad/s]
            j = i - 1
            if j == 0:
                x[j] = 0
                v_x[j] = 0
                y[j] = 0
                v_y[j] = 0
            else:
                v_x[j] = v_x[j - 1] + imu_data[0] * dt
                v_y[j] = v_y[j - 1] + imu_data[1] * dt
                x[j] = x[j - 1] + v_x[j] * dt
                y[j] = y[j - 1] + v_y[j] * dt
                theta[j] = theta[j - 1] + imu_data[2] * dt
            print(imu_data, x[j], y[j], theta[j])
        i += 1
# Close the serial port connection
ser.close()
plt.plot(x, y)
plt.show()
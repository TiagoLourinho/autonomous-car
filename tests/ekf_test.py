import numpy as np
import sys
import os

sys.path.insert(0, os.getcwd() + "/source")

# Ignore warning
from blocks import EKF

start = np.array([0, 0])

frequency = 100

ekf = EKF(start, frequency)

print("Commands:")
print("- p <linear velocity> <angular velocity in degrees>")
print("- u <sensor> <measurement 0> ... <measurement N> ")
print()

state = ekf.get_current_state()
cov = np.linalg.det(ekf.get_current_cov())
print(f"Initial State: {state}")
print(f"Initial Covariance determinant: {cov}")
print()
while True:
    command = input("$ ").split()

    # Predict
    if command[0].lower() == "p":
        v = float(command[1])
        omega = np.deg2rad(float(command[2]))

        ekf.predict(np.array([v, omega]))

    # Update
    elif command[0].lower() == "u":

        sensor = command[1]
        data = np.array([float(num) for num in command[2:]])

        ekf.update(data, sensor)

    state = ekf.get_current_state()
    cov = np.linalg.det(ekf.get_current_cov())

    print(f"State: {state}")
    print(f"Covariance determinant: {cov}")
    print()

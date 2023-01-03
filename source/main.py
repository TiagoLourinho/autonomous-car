from threading import Thread, Lock
import numpy as np
import matplotlib.pyplot as plt
from time import sleep

from blocks import EKF

OBJETIVE = np.array()  # Objetive position in lat/lon
TIME_STEP = 0.1  # ms

# Thread related
lock = Lock()
thread_shutdown = False


new_sensor_data = False  # Signals new sensor data available
ekf = EKF()  # Keeps and updates system state
current_control = np.array()  # Keeps the current controllers


def get_initial_position():
    """Reads the sensors and returns an initial position guess (x, y)"""

    return np.array()


def get_path(initial_position, objetive):
    """Returns a list with the points (path) from `initial_position` to `objetive`"""

    return np.array()


def check_new_sensor_data():
    """Checks for sensor data and updates EKF"""

    global thread_shutdown
    while not thread_shutdown:

        if new_sensor_data:
            new_sensor_data = False

            ekf.update()


def display_current_state(path):
    """Displays the path and the current state"""

    # Display initial path
    plt.plot(path)

    global thread_shutdown
    while not thread_shutdown:

        # Update gui with the car
        plt.plot(ekf.get_current_estimate())


def update_current_controls():
    """Updates the current controls acordding the current state and desired path"""

    global thread_shutdown
    while not thread_shutdown:
        current_control = np.array()


def send_controls(current_control):
    """Sends the current controls to the car and updates EKF"""

    ekf.predict(current_control)


def main():
    initial_position = get_initial_position()

    path = get_path(initial_position, OBJETIVE)

    threads = {
        "sensor_thread": Thread(target=check_new_sensor_data),
        "gui_thread": Thread(target=display_current_state, args=(path,)),
        "controller_thread": Thread(target=update_current_controls),
    }

    for t in threads:
        t.start()

    try:
        while True:
            send_controls(current_control)
            sleep(TIME_STEP)

    except KeyboardInterrupt:
        thread_shutdown = True

    finally:
        for t in threads:
            t.join()


if __name__ == "__main__":
    main()

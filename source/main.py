import time
from threading import Lock, Thread
from time import sleep
import traceback

import matplotlib.pyplot as plt
import numpy as np
from blocks import EKF, Car, Controller, Map, Sensors
from constants import *

FREQUENCY = 100  # Hz

# Thread related
lock = Lock()
thread_shutdown = False

update_gui = False  # Signals if the car moved

map = Map()

# Load the image file and convert it to a NumPy array
image = plt.imread("images/map_improved.png")

OBJETIVE = np.array([38.736911, -9.139010])  # Objetive position in lat/lon

# Get the map's corner coordinates
top_right_corner = map.get_coordinates(*TOP_RIGHT_CORNER)
bottom_left_corner = map.get_coordinates(BOTTOM_LEFT_CORNER[0], BOTTOM_LEFT_CORNER[1])


# Get the coordinates of the origin point
origin = map.get_coordinates(ORIGIN[0], ORIGIN[1]).reshape((2,))

ekf = EKF(origin, FREQUENCY)

fig, ax = plt.subplots()  # Create a figure and axes object

# Set the x-axis and y-axis limits to center the origin point
x_lims = [
    round(bottom_left_corner[0][0]) - round(origin[0]),
    round(top_right_corner[0][0]) - round(origin[0]),
]
y_lims = [
    round(bottom_left_corner[0][1]) - round(origin[1]),
    round(top_right_corner[0][1]) - round(origin[1]),
]
ax.set_xlim(x_lims[0], x_lims[1])
ax.set_ylim(y_lims[0], y_lims[1])

controller = Controller(
    qsi=1, w_n=10, v_ref=36, w_ref=4, h=0.01, L=2.2
)  # Control model for the steering wheel


def get_initial_position():
    """Reads the sensors and returns an initial position guess (x, y)"""

    return ORIGIN


def get_path(initial_position, objective):
    """Returns a list with the points (path) from `initial_position` to `objetive`"""
    coords = map.get_path(
        initial_position, objective
    )  # returns coordinates with (0, 0) being the bottom left corner of the map
    return coords


def check_new_sensor_data():
    """Checks for sensor data and updates EKF"""

    global thread_shutdown

    sensors = Sensors()
    # Because in simulation there's always data available, let's
    # define a limit to how frequently we can poll
    gps_poll_freq = 1
    imu_poll_freq = 0.01
    last_gps_poll = 0
    last_imu_poll = 0

    while not thread_shutdown:
        sensors.acquire()

        state = ekf.get_current_state()

        sensors.update_world_view(state[2], state[:2], state[4:6], np.array((0.0, 0.0)))

        pos = sensors.get_GPS_position()
        velocities = sensors.get_IMU_data()
        if pos is not None and time.time() - last_gps_poll >= gps_poll_freq:
            ekf.update(pos, "gps")
            last_gps_poll = time.time()
        if velocities is not None and time.time() - last_imu_poll >= imu_poll_freq:
            ekf.update(velocities, "imu")
            last_imu_poll = time.time()


def display_current_state(path):
    """Displays the path and the current state"""

    # Display initial path
    ax.imshow(
        image, extent=[x_lims[0], x_lims[1], y_lims[0], y_lims[1]]
    )  # Plot the image on the axes
    for i in range(len(path)):
        x = path[i][0] - round(origin[0])
        y = path[i][1] - round(origin[1])
        ax.plot(x, y, "ro", markersize=3)  # Plot the path on the axes

    global thread_shutdown
    global update_gui
    while not thread_shutdown:
        with lock:
            if update_gui:
                update_gui = False

                position = ekf.get_current_state()[:2]

                # Update gui with the car
                plt.plot(
                    position[0] - round(origin[0]),
                    position[1] - round(origin[1]),
                    "bo",
                    markersize=5,
                )  # Plot the car on the axes)


def get_control(next_point):
    """Updates the current controls acordding the current state and desired path"""
    global update_gui
    with lock:

        # integrate the sensor data

        pose = ekf.get_current_state()[:3]

        current_control = controller.following_trajectory(next_point, pose)
        ekf.predict(current_control)  # current_control
        update_gui = True


def main():
    initial_position = get_initial_position()

    path = get_path(initial_position, OBJETIVE)

    threads = {
        "sensor_thread": Thread(target=check_new_sensor_data),
        "gui_thread": Thread(target=display_current_state, args=(path,)),
    }

    for t in threads.values():
        t.start()

    try:
        for point in map.orient_path(path):
            while True:

                get_control(point)

                sleep(1 / FREQUENCY)

                position = ekf.get_current_state()[:2]

                if np.linalg.norm(position - point[:2]) < 2:
                    break

    except Exception:
        print(traceback.format_exc())
        thread_shutdown = True

    finally:
        for t in threads.values():
            t.join()


if __name__ == "__main__":
    main()

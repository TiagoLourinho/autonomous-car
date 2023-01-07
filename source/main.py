from threading import Thread, Lock
import numpy as np
import matplotlib.pyplot as plt
from time import sleep

from blocks import EKF, Map, Car
from constants import ORIGIN, TOP_LEFT_CORNER, TOP_RIGHT_CORNER, BOTTOM_LEFT_CORNER, BOTTOM_RIGHT_CORNER

OBJETIVE = np.array()  # Objetive position in lat/lon
TIME_STEP = 0.1  # ms

# Thread related
lock = Lock()
thread_shutdown = False


new_sensor_data = False  # Signals new sensor data available
ekf = EKF()  # Keeps and updates system state
current_control = np.array()  # Keeps the current controllers
map = Map()  # Keeps the map

# Load the image file and convert it to a NumPy array
image = plt.imread('images/map_improved.png')

# Get the map's corner coordinates
top_right_corner = map.get_coordinates(TOP_RIGHT_CORNER[0], TOP_RIGHT_CORNER[1])
bottom_left_corner = map.get_coordinates(BOTTOM_LEFT_CORNER[0], BOTTOM_LEFT_CORNER[1])
bottom_right_corner = map.get_coordinates(BOTTOM_RIGHT_CORNER[0], BOTTOM_RIGHT_CORNER[1])
top_left_corner = map.get_coordinates(TOP_LEFT_CORNER[0], TOP_LEFT_CORNER[1])

# Get the coordinates of the origin point
origin = map.get_coordinates(ORIGIN[0], ORIGIN[1])

fig, ax = plt.subplots()  # Create a figure and axes object

# Set the x-axis and y-axis limits to center the origin point
x_lims = [round(bottom_left_corner[0][0]) - round(origin[0][0]), round(top_right_corner[0][0]) - round(origin[0][0])]
y_lims = [round(bottom_left_corner[0][1]) - round(origin[0][1]), round(top_right_corner[0][1]) - round(origin[0][1])]
ax.set_xlim(x_lims[0], x_lims[1])
ax.set_ylim(y_lims[0], y_lims[1])


def get_initial_position():
    """Reads the sensors and returns an initial position guess (x, y)"""

    return np.array()


def get_path(initial_position, objective):
    """Returns a list with the points (path) from `initial_position` to `objetive`"""
    coords = map.get_path(initial_position, objective) # returns coordinates with (0, 0) being the bottom left corner of the map
    return coords


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
    ax.imshow(image, extent=[x_lims[0], x_lims[1], y_lims[0], y_lims[1]])  # Plot the image on the axes
    for i in range(len(path)):
        x = path[i][0] - round(origin[0][0])
        y = path[i][1] - round(origin[0][1])
        ax.plot(x, y, 'ro', markersize=3)  # Plot the path on the axes

    global thread_shutdown
    while not thread_shutdown:

        # Update gui with the car
        ax.plot(ekf.get_current_estimate(), 'bo', markersize=5)  # Plot the car on the axes


def update_current_controls():
    """Updates the current controls acordding the current state and desired path"""

    global thread_shutdown
    while not thread_shutdown:
        current_control = get_controls(ekf.get_current_estimate())


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

import time
from threading import Lock, Thread
from time import sleep
import traceback

import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np

from matplotlib.patches import Circle
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

OBJECTIVE = np.array([38.736911, -9.139010])  # Objetive position in lat/lon

# Get the map's corner coordinates
top_right_corner = map.get_coordinates(*TOP_RIGHT_CORNER)
bottom_left_corner = map.get_coordinates(BOTTOM_LEFT_CORNER[0], BOTTOM_LEFT_CORNER[1])


# Get the coordinates of the origin point
origin = map.get_coordinates(ORIGIN[0], ORIGIN[1]).reshape((2,))

ekf = EKF(origin, FREQUENCY)

# Set the x-axis and y-axis limits to center the origin point
x_lims = [
    round(bottom_left_corner[0][0]) - round(origin[0]),
    round(top_right_corner[0][0]) - round(origin[0]),
]
y_lims = [
    round(bottom_left_corner[0][1]) - round(origin[1]),
    round(top_right_corner[0][1]) - round(origin[1]),
]

controller = Controller(
    qsi=1, w_n=10, v_ref=36, w_ref=4, h=0.01, L=2.2
)  # Control model for the steering wheel


def sensor_thread():
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


def update_plot(n, state):
    """Updates the plot"""

    axes = state["artists"]["axes"]

    # Drive the car and get the result state
    position = ekf.get_current_state()[:3]

    # Update car position and orientation
    state["artists"]["car_position"].set_data(
        position[0] - round(origin[0]), position[1] - round(origin[1])
    )

    state["artists"]["car_theta"] = axes.arrow(
        position[0] - round(origin[0]),
        position[1] - round(origin[1]),
        10 * np.cos(position[2]),
        10 * np.sin(position[2]),
        head_width=2,
        head_length=2,
        fc="k",
        ec="k",
    )

    return [
        state["artists"]["car_position"],
        state["artists"]["car_theta"],
    ]


def display_current_state(path):
    """Displays the path and the current state"""
    state = {"artists": dict()}

    # Plot the point on the map
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
    ax.imshow(
        image, extent=[x_lims[0], x_lims[1], y_lims[0], y_lims[1]]
    )  # Plot the image on the axes

    for i in range(len(path)):
        x = path[i][0] - round(origin[0])
        y = path[i][1] - round(origin[1])
        ax.plot(x, y, "ro", markersize=3)  # Plot the path on the axes

    state["artists"]["axes"] = ax
    (state["artists"]["car_position"],) = ax.plot([], [], "bo", markersize=10)
    state["artists"]["car_theta"] = None

    anim = animation.FuncAnimation(
        fig,
        lambda n: update_plot(n, state),
        frames=None,
        interval=1000 / FREQUENCY,
        blit=True,
    )

    # Show the plot
    plt.show()


def control_thread(path):
    """Updates the current controls acordding the current state and desired path"""
    global update_gui
    global thread_shutdown
    for point in map.orient_path(path):
        while True:

            pose = ekf.get_current_state()[:3]
            current_control = controller.following_trajectory(point, pose)
            ekf.predict(current_control)

            with lock:
                update_gui = True

            sleep(1 / FREQUENCY)

            position = ekf.get_current_state()[:2]

            if np.linalg.norm(position - point[:2]) < 2:
                break

    thread_shutdown = True


def main():
    initial_position = ORIGIN

    path = map.get_path(
        initial_position, OBJECTIVE
    )  # returns coordinates with (0, 0) being the bottom left corner of the map

    threads = {
        "sensor_thread": Thread(target=sensor_thread),
        "controller_thread": Thread(target=control_thread, args=(path,)),
    }

    for t in threads.values():
        t.start()

    try:
        display_current_state(path)
    except Exception:
        print(traceback.format_exc())
        thread_shutdown = True

    finally:
        for t in threads.values():
            t.join()


if __name__ == "__main__":
    main()

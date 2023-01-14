import time
from threading import Lock, Thread
from time import sleep
import traceback

import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np


from blocks import EKF, Controller, Map, Sensors
from constants import *

# from blocks.mpc import MPC_Controller

START = np.array([38.737953, -9.138711])  # np.array([38.737151, -9.139810])

END = np.array([38.736911, -9.139010])  # Objetive position in lat/lon
FREQUENCY = 100  # Hz

# Thread related
lock = Lock()
thread_shutdown = False

# Blocks
map = Map()
controller = Controller(qsi=1, w_n=10, v_ref=36, w_ref=4, h=0.01, L=2.2)
# controller = MPC_Controller()
origin = map.get_coordinates(*ORIGIN).reshape((2,))


def sensor_thread():
    """Function to run in a thread, checking for new sensor data and updating EKF"""

    sleep(5)  # Load GUI

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

        sensors.update_world_view(state[2], state[:2], state[4:7], None)

        if time.time() - last_gps_poll >= gps_poll_freq:
            pos = sensors.get_GPS_position()
            ekf.update(pos, "gps")
            last_gps_poll = time.time()
        if time.time() - last_imu_poll >= imu_poll_freq:
            velocities = sensors.get_IMU_data()
            ekf.update(velocities, "imu")
            last_imu_poll = time.time()


def control_thread(oriented_path):
    """Function to run in a thread, calculating the control signals"""

    sleep(5)  # Load GUI

    global thread_shutdown

    for point in oriented_path:
        while True:

            pose = ekf.get_current_state()[:3]
            current_control = controller.following_trajectory(point, pose)
            ekf.predict(current_control)

            sleep(1 / FREQUENCY)

            position = ekf.get_current_state()[:2]

            # Move to next point if close enough to the current one
            if np.linalg.norm(position - point[:2]) < 3:  # Standard road width
                break

            if thread_shutdown:
                return

    # Terminate program when the objetive was reached
    thread_shutdown = True


def update_animation(n, state):
    """Updates the plot (used by FuncAnimation)"""

    axes = state["artists"]["axes"]

    position = ekf.get_current_state()[:3]

    state["artists"]["car_position"].set_data(
        position[0] - round(origin[0]), position[1] - round(origin[1])
    )

    state["artists"]["car_theta"] = axes.arrow(
        position[0] - round(origin[0]),
        position[1] - round(origin[1]),
        5 * np.cos(position[2]),
        5 * np.sin(position[2]),
        head_width=3,
        head_length=3,
        color="b",
    )

    return [
        state["artists"]["car_position"],
        state["artists"]["car_theta"],
    ]


def start_gui(path):
    """Displays the path and the"""

    state = {"artists": dict()}
    # image = plt.imread("images/map_improved.png")
    image = plt.imread("images/ist.jpeg")

    fig, ax = plt.subplots()

    top_right_corner = map.get_coordinates(*TOP_RIGHT_CORNER).reshape((2,))
    bottom_left_corner = map.get_coordinates(*BOTTOM_LEFT_CORNER).reshape((2,))

    x_lims = [
        round(bottom_left_corner[0]) - round(origin[0]),
        round(top_right_corner[0]) - round(origin[0]),
    ]
    y_lims = [
        round(bottom_left_corner[1]) - round(origin[1]),
        round(top_right_corner[1]) - round(origin[1]),
    ]

    ax.set_xlim(x_lims[0], x_lims[1])
    ax.set_ylim(y_lims[0], y_lims[1])

    ax.imshow(image, extent=[x_lims[0], x_lims[1], y_lims[0], y_lims[1]])

    # Plotting path
    for i in range(len(path)):
        x = path[i][0] - round(origin[0])
        y = path[i][1] - round(origin[1])
        ax.plot(x, y, "ro", markersize=3)

    state["artists"]["axes"] = ax
    (state["artists"]["car_position"],) = ax.plot([], [], "bo", markersize=5)
    state["artists"]["car_theta"] = None

    anim = animation.FuncAnimation(
        fig,
        lambda n: update_animation(n, state),
        frames=None,
        interval=100,
        blit=True,
    )

    plt.show()


def main():
    global thread_shutdown
    global ekf

    path = map.get_path(START, END)
    oriented_path = map.orient_path(path)

    # Set initial theta
    ekf = EKF(
        np.concatenate(
            [map.get_coordinates(*START).reshape((2,)), [oriented_path[0][2]]]
        ),
        FREQUENCY,
    )

    threads = {
        "sensor_thread": Thread(target=sensor_thread),
        "controller_thread": Thread(target=control_thread, args=(oriented_path,)),
    }

    for t in threads.values():
        t.start()

    try:
        start_gui(path)
    except Exception:
        print(traceback.format_exc())
    finally:
        thread_shutdown = True
        for t in threads.values():
            t.join()


if __name__ == "__main__":
    main()

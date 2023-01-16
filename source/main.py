import time
from threading import Lock, Thread
from time import sleep
import traceback

import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np


from blocks import EKF, Controller, Map, Sensors
from blocks.mpc import MPC_Controller
from constants import *

from blocks.mpc import MPC_Controller


FREQUENCY = 100  # Hz

# Thread related
lock = Lock()
thread_shutdown = False

# Blocks
map = Map()
<<<<<<< HEAD
controller = Controller(qsi=1, w_n=10, v_ref=36, w_ref=4, h=0.01, L=2.46)
# controller = MPC_Controller()
=======
controller = Controller(qsi=1, w_n=10, v_ref=36, w_ref=4, h=0.01, L=2.2)
#controller = MPC_Controller()
>>>>>>> 7f71006 (fixing conflicts)
origin = map.get_coordinates(*ORIGIN).reshape((2,))


def sensor_thread(ekf):
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
            pos = state[:2]
            ekf.update(pos, "gps")
            last_gps_poll = time.time()
        if time.time() - last_imu_poll >= imu_poll_freq:
            velocities = sensors.get_IMU_data()
            velocities = state[4:7]
            ekf.update(velocities, "imu")
            last_imu_poll = time.time()


def control_thread(oriented_path, ekf):
    """Function to run in a thread, calculating the control signals"""

    sleep(5)  # Load GUI

    global thread_shutdown

    i=0
    for point in oriented_path:
        i+=1
        while True:
            if i==2:
                return

            pose = ekf.get_current_state()[:4]
            current_control = controller.following_trajectory(point, pose)

            ekf.predict(current_control)

            sleep(1 / FREQUENCY)

            position = ekf.get_current_state()[:2]

            if thread_shutdown:
                return

    # Terminate program when the objetive was reached
    thread_shutdown = True


def update_animation(n, state, ekf):
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


def start_gui(path, ekf):
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
    (state["artists"]["car_position"],) = ax.plot(
        *ekf.get_current_state()[:2], "bo", markersize=5
    )
    state["artists"]["car_theta"] = None

    anim = animation.FuncAnimation(
        fig,
        lambda n: update_animation(n, state, ekf),
        frames=None,
        interval=100,
        blit=True,
    )

    plt.show()


def choose_path():
    def onclick(event, fig, points):
        if points["start"] is None:
            points["start"] = (event.xdata, event.ydata)

            # Update plot
            fig.gca().set_title("Click in the end position")
            fig.canvas.draw()
            fig.canvas.flush_events()

        elif points["end"] is None:
            points["end"] = (event.xdata, event.ydata)
            plt.close(fig)

    fig, ax = plt.subplots()

    image = plt.imread("images/ist.jpeg")

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

    ax.set_title("Click in the start position")

    ax.imshow(image, extent=[x_lims[0], x_lims[1], y_lims[0], y_lims[1]])

    points = {"start": None, "end": None}
    cid = fig.canvas.mpl_connect(
        "button_press_event", lambda event: onclick(event, fig, points)
    )

    plt.show()
    
    points["start"] /= np.array([map.scale_x, map.scale_y])
    points["end"] /= np.array([map.scale_x, map.scale_y])
    
    points["start"] = map.transformer.invtransform(points["start"][0], points["start"][1])
    points["end"] = map.transformer.invtransform(points["end"][0], points["end"][1])
    
    return map.get_path(points["start"], points["end"]), points["start"]


def main():
    global thread_shutdown

    path, start = choose_path()
    oriented_path = map.orient_path(path)

    # Set initial theta
    ekf = EKF(
        np.concatenate(
            [map.get_coordinates(*start).reshape((2,)), [oriented_path[0][2]]]
        ),
        FREQUENCY,
    )

    threads = {
        "sensor_thread": Thread(target=sensor_thread, args=(ekf,)),
        "controller_thread": Thread(target=control_thread, args=(oriented_path, ekf)),
    }

    for t in threads.values():
        t.start()

    try:
        start_gui(path, ekf)
    except Exception:
        print(traceback.format_exc())
    finally:
        thread_shutdown = True
        for t in threads.values():
            t.join()


if __name__ == "__main__":
    main()

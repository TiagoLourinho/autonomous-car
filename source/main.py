import time
from threading import Lock, Thread
from time import sleep
import traceback
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
from scipy import signal

from blocks import (
    EKF,
    Controller,
    Map,
    Sensors,
    VelocityController,
    MotorController,
    MPC_Controller,
)

from constants import *


FREQUENCY = 100  # Hz
SIMULATION = True

# Thread related
lock = Lock()
thread_shutdown = False

# Blocks
map = Map()

# controller = Controller(qsi=1, w_n=10, v_ref=36, w_ref=4, h=0.01, L=2.46)
# controller = MPC_Controller()

origin = map.get_coordinates(*ORIGIN).reshape((2,))
control_signals = [[], []]


def sensor_thread(ekf):
    """Function to run in a thread, checking for new sensor data and updating EKF"""

    sleep(5)  # Load GUI

    global thread_shutdown

    sensors = Sensors(simulated=SIMULATION)

    # Because in simulation there's always data available, let's
    # define a limit to how frequently we can poll
    gps_poll_freq = 1
    imu_poll_freq = 0.01
    last_gps_poll = 0
    last_imu_poll = 0

    while not thread_shutdown:

        sensors.acquire()

        # The estimated state is no longer needed here, left it behind just to be clear about the changes made
        estimated_state = ekf.get_current_state()
        real_state = ekf.get_predicted_state()

        sensors.update_world_view(
            real_state[2].copy(), real_state[:2].copy(), real_state[4:7].copy(), None
        )

        if time.time() - last_gps_poll >= gps_poll_freq:
            pos = sensors.get_GPS_position()

            #pos = estimated_state[:2]

            ekf.update(pos, "gps")
            last_gps_poll = time.time()
        if time.time() - last_imu_poll >= imu_poll_freq:
            velocities = sensors.get_IMU_data()

            #velocities = estimated_state[4:7]

            ekf.update(velocities, "imu")
            last_imu_poll = time.time()


def control_thread(oriented_path, ekf, controller, motor_controller):
    """Function to run in a thread, calculating the control signals"""

    sleep(5)  # Load GUI

    global thread_shutdown
    global control_signals
    M = 1190
    P0 = 500
    positions = []
    energy_used = 0
    energy_usage = []
    j = -1
    already_filtered = 0
    for i, point in enumerate(oriented_path):
        while True:
            position = ekf.get_current_state()[:2]
            positions.append(position)
            j += 1
            # Move to next point if close enough to the current one
            if (
                np.linalg.norm(position - point[:2]) < 4 and i <= len(oriented_path) - 3
            ):  # Standard road width
                break
            elif (
                np.linalg.norm(position - point[:2]) < 1 and i > len(oriented_path) - 3
            ):
                break
            
            

            pose = ekf.get_current_state()[:6]
            current_control = controller.following_trajectory(point, pose, energy_used)

            # Filtering (Max steering angle)
            phi = pose[3]
            omega = current_control[1]

            if (omega > 0 and phi == ekf.get_max_steering_angle()) or (
                omega < 0 and phi == -ekf.get_max_steering_angle()
            ):
                current_control[1] = 0

            # Filtering (low pass)
            if already_filtered == 0:
                b = signal.firwin(80, 0.004)
                z = signal.lfilter_zi(b, 1) * current_control[1]
                current_control[1], z = signal.lfilter(b, 1, [current_control[1]], zi=z)
                already_filtered += 1
            else:
                current_control[1], z = signal.lfilter(b, 1, [current_control[1]], zi=z)

            control_signals[0].append(current_control.tolist()[0])
            control_signals[1].append(current_control.tolist()[1])

            # Send commands
            motor_controller.send_control(phi, current_control)
            ekf.predict(current_control)

            sleep(1 / FREQUENCY)

            # Energy usage
            d = np.linalg.norm(position - positions[j - 1])
            v = np.sqrt(pose[4] ** 2 + pose[5] ** 2)
            energy_used += M * d * v + P0 * (1 / FREQUENCY)
            energy_usage.append(energy_used)

            if thread_shutdown:
                return
    # Terminate program when the objetive was reached
    thread_shutdown = True


def update_animation(n, state, ekf):
    """Updates the plot (used by FuncAnimation)"""

    axes = state["artists"]["axes"]

    estimated_position = ekf.get_current_state()[:3]

    real_position = ekf.get_predicted_state()[:3]

    state["artists"]["estimated_car_position"].set_data(
        estimated_position[0] - round(origin[0]),
        estimated_position[1] - round(origin[1]),
    )

    state["artists"]["estimated_car_theta"] = axes.arrow(
        estimated_position[0] - round(origin[0]),
        estimated_position[1] - round(origin[1]),
        5 * np.cos(estimated_position[2]),
        5 * np.sin(estimated_position[2]),
        head_width=3,
        head_length=3,
        color="b",
    )

    state["artists"]["real_car_position"].set_data(
        real_position[0] - round(origin[0]), real_position[1] - round(origin[1])
    )

    state["artists"]["real_car_theta"] = axes.arrow(
        real_position[0] - round(origin[0]),
        real_position[1] - round(origin[1]),
        5 * np.cos(real_position[2]),
        5 * np.sin(real_position[2]),
        head_width=3,
        head_length=3,
        color="g",
    )

    return [
        state["artists"]["estimated_car_position"],
        state["artists"]["estimated_car_theta"],
        state["artists"]["real_car_position"],
        state["artists"]["real_car_theta"],
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
    (state["artists"]["estimated_car_position"],) = ax.plot(
        *ekf.get_current_state()[:2], "bo", markersize=5, label="Estimated Pose"
    )
    state["artists"]["estimated_car_theta"] = None
    (state["artists"]["real_car_position"],) = ax.plot(
        *ekf.get_predicted_state()[:2],
        "go",
        markersize=5,
        label="Predicted Pose (car model)",
    )
    state["artists"]["real_car_theta"] = None
    ax.legend()
    ax.set_title("Simulating..." if SIMULATION else "Status")

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
            fig.gca().scatter(event.xdata, event.ydata, label="Start", c="g")
            plt.legend()
            fig.gca().set_title("Click in the end position")
            fig.canvas.draw()
            fig.canvas.flush_events()

        elif points["end"] is None:
            points["end"] = (event.xdata, event.ydata)

            # Update plot
            fig.gca().scatter(event.xdata, event.ydata, label="End", c="r")
            plt.legend()
            fig.gca().set_title("Looking for a valid path...")
            fig.canvas.draw()
            fig.canvas.flush_events()
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

    points["start"] = map.transformer.invtransform(
        points["start"][0], points["start"][1]
    )
    points["end"] = map.transformer.invtransform(points["end"][0], points["end"][1])

    if map.verify_point(points["start"]) and map.verify_point(points["end"]):
        print("Valid path")
    else:
        print("Invalid path")
        exit(1)

    return map.get_path(points["start"], points["end"])


def main():
    global thread_shutdown
    path = choose_path()
    path =  map.round_path(path)
    # Remove duplicate points
    _, idx = np.unique(path, axis=0, return_index=True)
    path = path[np.sort(idx)]

    oriented_path = map.orient_path(path)

    cont = VelocityController(path)
    motor_controller = MotorController(FREQUENCY, SIMULATION)

    # Set initial theta
    ekf = EKF(
        np.concatenate([path[0], [oriented_path[0][2]]]),
        FREQUENCY,
    )

    threads = {
        "sensor_thread": Thread(target=sensor_thread, args=(ekf,)),
        "controller_thread": Thread(
            target=control_thread, args=(oriented_path, ekf, cont, motor_controller)
        ),
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

    time = np.arange(0, len(control_signals[0]), 1)
    time = time * 1 / FREQUENCY
    plt.figure()
    plt.plot(time, control_signals[0])
    plt.grid(True)
    plt.xlabel(f"Time [s]")
    plt.ylabel("V [m/s]")
    plt.figure()
    plt.plot(time, control_signals[1])
    plt.grid(True)
    plt.xlabel(f"Time [s]")
    plt.ylabel(r"$\omega_{s}$ [rad/s]")
    plt.show()


if __name__ == "__main__":
    main()

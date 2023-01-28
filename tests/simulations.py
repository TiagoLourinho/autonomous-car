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

# Thread related
lock = Lock()
thread_shutdown = False

# Blocks
map = Map()

# controller = Controller(qsi=1, w_n=10, v_ref=36, w_ref=4, h=0.01, L=2.46)
# controller = MPC_Controller()

origin = map.get_coordinates(*ORIGIN).reshape((2,))
control_signals = [[], []]

# PUT THIS ENERGY FUNCTION IN AN APPROPRIATE PLACE
def update_energy_usage(
    curr_idx: int,
    positions: list,
    pose: np.array,
    true_position: np.array,
    freq: float,
    M: float,
    P0: float,
    multiplier: float,
):
    if curr_idx >= 1:
        d = np.linalg.norm(true_position - positions[curr_idx - 1])
        v = np.sqrt(pose[4] ** 2 + pose[5] ** 2)
        return multiplier * M * d * v + P0 * (1 / freq)
    else:
        return 0


def sensor_thread(ekf):
    """Function to run in a thread, checking for new sensor data and updating EKF"""

    sleep(5)  # Load GUI

    global thread_shutdown

    sensors = Sensors(simulated=True)

    # Because in simulation there's always data available, let's
    # define a limit to how frequently we can poll
    gps_poll_freq = 1
    imu_poll_freq = 0.01
    last_gps_poll = 0
    last_imu_poll = 0
    try:
        while not thread_shutdown:

            sensors.acquire()

            # The estimated state is no longer needed here, left it behind just to be clear about the changes made
            estimated_state = ekf.get_current_state()
            real_state = ekf.get_predicted_state()

            sensors.update_world_view(
                real_state[2].copy(),
                real_state[:2].copy(),
                real_state[4:7].copy(),
                None,
            )

            if time.time() - last_gps_poll >= gps_poll_freq:
                pos = sensors.get_GPS_position()

                # pos = estimated_state[:2]

                ekf.update(pos, "gps")
                last_gps_poll = time.time()
            if time.time() - last_imu_poll >= imu_poll_freq:
                velocities = sensors.get_IMU_data()

                # velocities = estimated_state[4:7]

                ekf.update(velocities, "imu")
                last_imu_poll = time.time()
    except Exception:
        print(traceback.format_exc())
    finally:
        thread_shutdown = True


def control_thread(
    oriented_path,
    ekf,
    controller,
    motor_controller,
    energies,
    max_steering,
    avg_vel_error,
):
    """Function to run in a thread, calculating the control signals"""

    sleep(5)  # Load GUI

    global thread_shutdown
    global control_signals
    positions = []
    energy_used = 0
    energy_usage = []
    j = -1
    already_filtered = 0

    try:
        for i, point in enumerate(oriented_path):
            while True:
                position = ekf.get_current_state()[:2]

                # Retrieve true position for energy calculation purpose only
                true_position = ekf.get_predicted_state()[:2]
                positions.append(true_position)
                j += 1

                # Move to next point if close enough to the current one
                if (
                    np.linalg.norm(position - point[:2]) < 5
                    and i <= len(oriented_path) - 3
                ):  # Standard road width
                    break
                elif (
                    np.linalg.norm(position - point[:2]) < 1
                    and i > len(oriented_path) - 3
                ):
                    break

                pose = ekf.get_current_state()[:6]
                current_control = controller.following_trajectory(
                    point, pose, energy_used, i - 1
                )

                # PUT FILTERING INTO FUNCTION in appropriate place
                # Filtering (Max steering angle)
                phi = pose[3]
                omega = current_control[1]

                if (omega > 0 and phi >= ekf.get_max_steering_angle()) or (
                    omega < 0 and phi <= -ekf.get_max_steering_angle()
                ):
                    current_control[1] = 0

                # Filtering (low pass)
                if already_filtered == 0:
                    b = signal.firwin(80, 0.004)
                    z = signal.lfilter_zi(b, 1) * current_control[1]
                    current_control[1], z = signal.lfilter(
                        b, 1, [current_control[1]], zi=z
                    )
                    already_filtered += 1
                else:
                    current_control[1], z = signal.lfilter(
                        b, 1, [current_control[1]], zi=z
                    )

                control_signals[0].append(current_control.tolist()[0])
                control_signals[1].append(current_control.tolist()[1])

                # Send commands
                phi = motor_controller.send_control(phi, current_control)
                ekf.predict(current_control)

                if phi is not None:  # Controller gives the true phi
                    ekf.set_state(3, phi)

                sleep(1 / FREQUENCY)

                # Energy usage
                current_energy = update_energy_usage(
                    j, positions, pose, true_position, FREQUENCY, M, P0, multiplier
                )
                energy_used += current_energy
                energy_usage.append(energy_used)

                if thread_shutdown:
                    return
        print("END OF RUN")
        energies.append(energy_used)
        max_steering.append(max(control_signals[1]))
        avg_vel_error.append(controller.get_full_path_vel_error())
        plt.close("all")

    except Exception:
        print(traceback.format_exc())
    finally:
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
    return fig


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


def get_best_setup(
    energies: None, max_steering: None, avg_velocity_errors: None, kws: list, kvs: list
):
    """
    Computes cost function and best controllers parameters
    """
    # Hard coded example
    if not energies or not max_steering or not avg_velocity_errors:
        energies = [
            60912.673082858135,
            56758.88096067548,
            58819.34940821276,
            58879.965555220915,
            56905.25401275503,
            417454.87569244637,
            97758.22335860803,
            90550.29901756142,
            87429.81500117146,
            91002.90140013302,
            121278.39874246097,
            333547.16842112556,
            120969.11967135881,
            328999.44007044774,
            151668.4863952779,
            294914.33393501554,
            138063.07325786314,
            243239.1239302971,
            256802.09936381428,
            528252.9130274684,
        ]
        max_steering = [
            0.22041453903047506,
            0.32132989243581594,
            0.5355160912443787,
            0.7464958925376739,
            1.1806202334344325,
            1.1806202334344325,
            1.1806202334344325,
            1.1806202334344325,
            1.1806202334344325,
            1.2114907089527192,
            1.2114907089527192,
            1.2114907089527192,
            1.2114907089527192,
            1.2114907089527192,
            1.2114907089527192,
            1.2114907089527192,
            1.2114907089527192,
            1.2114907089527192,
            1.2114907089527192,
            1.2114907089527192,
        ]
        avg_velocity_errors = [
            18176.709260904532,
            15234.651685520392,
            14370.275665382916,
            15017.269657155135,
            14014.462739301021,
            27635.238907840234,
            9634.912432579871,
            10431.093669493643,
            7718.999438907495,
            9046.939499081123,
            4356.098046263925,
            10895.91224168289,
            5875.05802210517,
            11435.985496916304,
            4145.87534726199,
            73865.33444736328,
            4941.891492391183,
            3254.6485713275047,
            5039.108002306373,
            15192.077540346621,
        ]

    # Normalizing metrics
    normal_energies = [el / (sum(energies) / len(energies)) for el in energies]
    normal_max_steering = [
        el / (sum(max_steering) / len(max_steering)) for el in max_steering
    ]
    normal_avg_velocity_errors = [
        el / (sum(avg_velocity_errors) / len(avg_velocity_errors))
        for el in avg_velocity_errors
    ]

    # Computing cost
    C = []
    for i in range(len(energies)):
        C.append(
            normal_energies[i] + normal_max_steering[i] + normal_avg_velocity_errors[i]
        )
    best_index = C.index(min(C))
    print(f"Minimum cost was {min(C)} which corresponds to index {best_index}")
    kv_index = (best_index + 1) // len(kvs)
    kw_index = (best_index + 1) % len(kws)
    print(f"Best Kv: {kvs[kv_index]}")
    print(f"Best Kw: {kws[kw_index-1]}")


def main(path, oriented_path, kw, kv, energies, max_steering, avg_vel_error):
    global thread_shutdown

    cont = VelocityController(path, kw, kv)
    motor_controller = MotorController(FREQUENCY, SIMULATION)

    # Set initial theta
    ekf = EKF(
        np.concatenate([path[0], [oriented_path[0][2]]]),
        FREQUENCY,
    )

    threads = {
        "sensor_thread": Thread(target=sensor_thread, args=(ekf,)),
        "controller_thread": Thread(
            target=control_thread,
            args=(
                oriented_path,
                ekf,
                cont,
                motor_controller,
                energies,
                max_steering,
                avg_vel_error,
            ),
        ),
    }

    for t in threads.values():
        t.start()

    try:
        plot = start_gui(path, ekf)
    except Exception:
        print(traceback.format_exc())
    finally:
        thread_shutdown = True
        for t in threads.values():
            t.join()
        motor_controller.housekeeping()

    time = np.arange(0, len(control_signals[0]), 1)
    time = time * 1 / FREQUENCY
    plt.figure()
    plt.plot(time, control_signals[0])
    plt.grid(True)
    plt.xlabel(f"Time [s]")
    plt.ylabel("V [m/s]")
    plt.savefig(f"source/simulations/Vs/kw{kw}_kv{kv}.png")
    plt.figure()
    plt.plot(time, control_signals[1])
    plt.grid(True)
    plt.xlabel(f"Time [s]")
    plt.ylabel(r"$\omega_{s}$ [rad/s]")
    # plt.show()
    plt.savefig(f"source/simulations/ws/kw{kw}_kv{kv}.png")


if __name__ == "__main__":
    run_simulations = 0

    # Choose values of Kv and Kw to test
    Kvs = np.linspace(0.2, 1.2, 5)
    Kws = np.linspace(0.2, 1.2, 5)

    # Cost function variables to save
    energies = []
    max_steering = []
    avg_vel_errors = []

    if not run_simulations:
        get_best_setup(energies, max_steering, avg_vel_errors, Kws, Kvs)
        exit()

    # Choose Path
    path = choose_path()
    path = map.round_path(path)
    # Remove duplicate points
    _, idx = np.unique(path, axis=0, return_index=True)
    path = path[np.sort(idx)]
    oriented_path = map.orient_path(path)

    # Choose hardcoded path
    # path = np.array([np.array([ -50.02986682,  -92.29456594]), np.array([ -49.89549942,  -94.30868594]), np.array([ -49.76097808,  -96.32055392]), np.array([ -49.62640911,  -98.33313425]), np.array([ -49.49188777, -100.34500214]), np.array([ -49.35731617, -102.35762174]), np.array([ -48.31689076, -105.10123421]), np.array([ -44.82788839, -106.01495987]), np.array([ -42.69775178, -105.83140772]), np.array([ -40.56528764, -105.66476028]), np.array([ -38.4318056 , -105.52587587]), np.array([ -36.29827504, -105.38698829]), np.array([ -34.16479315, -105.24810388])])
    # oriented_path = np.array([np.array([-5.00298668e+01, -9.22945659e+01, -1.50418233e+00, -1.50418233e+00]), np.array([-4.98954994e+01, -9.43086859e+01, -1.50403180e+00, -1.50403180e+00]), np.array([-4.97609781e+01, -9.63205539e+01, -1.50403180e+00, -1.50403180e+00]), np.array([-4.96264091e+01, -9.83331343e+01, -1.50403180e+00, -1.50403180e+00]), np.array([-4.94918878e+01, -1.00345002e+02, -1.50403180e+00, -1.50403180e+00]), np.array([-4.93573162e+01, -1.02357622e+02, -1.20833340e+00, -1.20833340e+00]), np.array([-4.83168908e+01, -1.05101234e+02, -2.56135107e-01, -2.56135107e-01]), np.array([-4.48278884e+01, -1.06014960e+02,  8.59568621e-02,  8.59568621e-02]), np.array([-4.26977518e+01, -1.05831408e+02,  7.79893230e-02,  7.79893230e-02]), np.array([-4.05652876e+01, -1.05664760e+02,  6.50058105e-02,  6.50058105e-02]), np.array([-3.84318056e+01, -1.05525876e+02,  6.50058126e-02,  6.50058126e-02]), np.array([-3.62982750e+01, -1.05386988e+02,  6.50058144e-02,  6.50058144e-02]), np.array([-3.41647931e+01, -1.05248104e+02,  6.50058144e-02,  6.50058144e-02])])

    for kv in Kvs:
        for kw in Kws:
            main(path, oriented_path, kw, kv, energies, max_steering, avg_vel_errors)
            thread_shutdown = False
            print("End of main()")

    get_best_setup(energies, max_steering, avg_vel_errors, kw, kv)

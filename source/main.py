import numpy as np
import matplotlib.animation as animation
import matplotlib.pyplot as plt

from blocks import Map, Car
from constants import ORIGIN

##### Hyperparameters #####

TIME_STEP = 0.1  # seconds
LINEAR_VELOCITY = 10  # kilometer per hour
STEERING_VELOCITY = np.pi / 12  # radians per second


def update(n, state):
    axes = state["artists"]["axes"]

    # Drive the car and get the result state
    state["car"].drive(LINEAR_VELOCITY * 0.277777778, STEERING_VELOCITY, TIME_STEP)
    current_state = state["car"].get_current_state()
    length = state["car"].get_length()

    # Update car position and orientation
    state["artists"]["car_position"].set_data(current_state[0], current_state[1])

    state["artists"]["car_theta"] = axes.arrow(
        current_state[0],
        current_state[1],
        2 * length * np.cos(current_state[2]),
        2 * length * np.sin(current_state[2]),
        width=0.5,
        color="b",
    )

    state["artists"]["car_phi"] = axes.arrow(
        current_state[0] + length * np.cos(current_state[2]),
        current_state[1] + length * np.sin(current_state[2]),
        length * np.cos(current_state[3] + current_state[2]),
        length * np.sin(current_state[3] + current_state[2]),
        width=0.5,
        color="g",
    )

    return [
        state["artists"]["car_position"],
        state["artists"]["car_theta"],
        state["artists"]["car_phi"],
    ]


def main():
    fig = plt.figure()
    axes = plt.axes(xlim=(-20, 20), ylim=(-20, 20))
    axes.set_aspect("equal", "box")

    state = {"map": Map(), "car": Car(), "artists": dict()}

    # Set initial state
    state["artists"]["axes"] = axes
    (state["artists"]["car_position"],) = axes.plot([], [], "ro")
    state["artists"]["car_theta"] = None
    state["artists"]["car_phi"] = None

    anim = animation.FuncAnimation(
        fig,
        lambda n: update(n, state),
        frames=None,
        interval=TIME_STEP * 1000,
        blit=True,
    )
    plt.show()


if __name__ == "__main__":
    main()

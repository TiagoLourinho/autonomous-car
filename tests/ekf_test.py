import numpy as np
import sys
import os
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse
import matplotlib.transforms as transforms

sys.path.insert(0, os.getcwd() + "/source")

# Ignore warning
from blocks import EKF

start = np.array([0, 0])

frequency = 100  # Hz


def confidence_ellipse(x, y, ax, n_std=3.0, facecolor="none", **kwargs):
    """
    Create a plot of the covariance confidence ellipse of *x* and *y*.

    Parameters
    ----------
    x, y : array-like, shape (n, )
        Input data.

    ax : matplotlib.axes.Axes
        The axes object to draw the ellipse into.

    n_std : float
        The number of standard deviations to determine the ellipse's radiuses.

    **kwargs
        Forwarded to `~matplotlib.patches.Ellipse`

    Returns
    -------
    matplotlib.patches.Ellipse
    """
    if x.size != y.size:
        raise ValueError("x and y must be the same size")

    cov = np.cov(x, y)
    pearson = cov[0, 1] / np.sqrt(cov[0, 0] * cov[1, 1])
    # Using a special case to obtain the eigenvalues of this
    # two-dimensional dataset.
    ell_radius_x = np.sqrt(1 + pearson)
    ell_radius_y = np.sqrt(1 - pearson)
    ellipse = Ellipse(
        (0, 0),
        width=ell_radius_x * 2,
        height=ell_radius_y * 2,
        facecolor=facecolor,
        **kwargs,
    )

    # Calculating the standard deviation of x from
    # the squareroot of the variance and multiplying
    # with the given number of standard deviations.
    scale_x = np.sqrt(cov[0, 0]) * n_std
    mean_x = np.mean(x)

    # calculating the standard deviation of y ...
    scale_y = np.sqrt(cov[1, 1]) * n_std
    mean_y = np.mean(y)

    transf = (
        transforms.Affine2D()
        .rotate_deg(45)
        .scale(scale_x, scale_y)
        .translate(mean_x, mean_y)
    )

    ellipse.set_transform(transf + ax.transData)
    return ax.add_patch(ellipse)


def get_correlated_dataset(n, dependency, mu, scale):
    latent = np.random.randn(n, 2)
    dependent = latent.dot(dependency)
    scaled = dependent * scale
    scaled_with_offset = scaled + mu
    # return x and y of the new, correlated dataset
    return scaled_with_offset[:, 0], scaled_with_offset[:, 1]


def main(start, frequency):

    ekf = EKF(start, frequency)

    # Usage
    print("Commands:")
    print("- p <linear velocity> <angular velocity in degrees>")
    print("- u <sensor> <measurement 0> ... <measurement N> ")
    print()

    length = 10  # Arrows
    spacing = 10  # State prints

    # Axes limits
    center = start
    offset = 50

    plt.ion()
    while True:

        # Get current state
        state = [round(num, 2) for num in ekf.get_current_state()]
        cov = ekf.get_current_cov()

        # Prints
        print("=" * spacing * 8)
        print(
            f'{"x":^{spacing}}{"y":^{spacing}}{"theta":^{spacing}}{"phi":^{spacing}}{"x dot":^{spacing}}{"y dot":^{spacing}}{"theta dot":^{spacing}}{"phi dot":^{spacing}}'
        )
        print(
            f"{state[0]:^{spacing}}{state[1]:^{spacing}}{state[2]:^{spacing}}{state[3]:^{spacing}}{state[4]:^{spacing}}{state[5]:^{spacing}}{state[6]:^{spacing}}{state[7]:^{spacing}}"
        )
        print("=" * spacing * 8)

        # Adjust limits
        if (
            np.abs(state[0]) > center[0] + offset
            or np.abs(state[1]) > center[1] + offset
        ):
            center = state[:2]

        # Plot
        plt.gca().clear()
        plt.xlim(center[0] - offset, center[0] + offset)
        plt.ylim(center[1] - offset, center[1] + offset)
        plt.grid()

        x, y = get_correlated_dataset(800, cov[:2, :2], state[:2], 1)
        confidence_ellipse(x, y, plt.gca(), facecolor="pink", edgecolor="purple")
        plt.plot(state[0], state[1], "ko", markersize=5)
        plt.arrow(
            state[0],
            state[1],
            2 * length * np.cos(state[2]),
            2 * length * np.sin(state[2]),
            width=0.5,
            color="b",
        )
        plt.arrow(
            state[0] + length * np.cos(state[2]),
            state[1] + length * np.sin(state[2]),
            length * np.cos(state[3] + state[2]),
            length * np.sin(state[3] + state[2]),
            width=0.5,
            color="g",
        )

        # EKF step
        try:
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
        except IndexError:
            continue
        except KeyboardInterrupt:
            plt.close()
            return


if __name__ == "__main__":
    main(start, frequency)

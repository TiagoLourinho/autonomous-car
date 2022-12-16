import numpy as np
import matplotlib.pyplot as plt

from blocks import Map
from constants import ORIGIN


def main():
    map = Map()

    civil = np.array([38.73726, -9.13984])

    points = map.get_directions(ORIGIN, civil)

    plt.plot(
        points[:, 1],
        points[:, 0],
    )
    plt.show()


if __name__ == "__main__":
    main()

import numpy as np
import math


def dms2deg(deg: int, min: int, sec: float, dir: str):
    """Convert degree, minutes and seconds plus direction to degrees

    Parameters
    ----------

    deg: int
        Degrees

    min: int
        Minutes

    sec: float
        Seconds

    dir: str
        Direction

    Returns
    -------

    float
        Converted angle in degrees
    """

    if dir == "N" or dir == "E":
        return deg + min / 60 + sec / 3600
    elif dir == "S" or dir == "W":
        return -(deg + min / 60 + sec / 3600)
    else:
        print("Please enter a valid dir")


def R_x(angle: float):
    """Rotation matix by `angle` on x-axis

    Parameters
    ----------

    angle: float
        Angle in radians

    Returns
    -------

    np.array of shape (3, 3)
        Rotation matrix
    """
    return np.array(
        [
            [1, 0, 0],
            [0, math.cos(angle), -math.sin(angle)],
            [0, math.sin(angle), math.cos(angle)],
        ]
    )


def R_y(angle):
    """Rotation matix by `angle` on y-axis

    Parameters
    ----------

    angle: float
        Angle in radians

    Returns
    -------

    np.array of shape (3, 3)
        Rotation matrix
    """
    return np.array(
        [
            [math.cos(angle), 0, math.sin(angle)],
            [0, 1, 0],
            [-math.sin(angle), 0, math.cos(angle)],
        ]
    )


def R_z(angle):
    """Rotation matix by `angle` on z-axis

    Parameters
    ----------

    angle: float
        Angle in radians

    Returns
    -------

    np.array of shape (3, 3)
        Rotation matrix
    """
    return np.array(
        [
            [math.cos(angle), -math.sin(angle), 0],
            [math.sin(angle), math.cos(angle), 0],
            [0, 0, 1],
        ]
    )


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

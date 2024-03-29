import numpy as np
import matplotlib.pyplot as plt

from utils import dms2deg

# Main parameters
FREQUENCY = 100  # Frequency operation in Hz
SIMULATION = True  # If it is a simulation or is using real sensors and motors

# Car constants
M = 1190  # kg
P0 = 500  # W
L = 2.46  # m

# Controller constants
Kw = 0.45
Kv = 0.9

# Energy constants
multiplier = 1.3
avg_velocity = 1.9  # For budget calculation
max_velocity = 2  # For energy-based optimization

# Map constants
GMAPS_KEY = "XXXXXXXXXXXXXXXXXXXXXXXXXXXX"  # Google Maps API key

ORIGIN = np.array(
    [38.73761835946306, -9.138958625673242]
)  # Latitude and Longitude of entrance of north tower

TOP_LEFT_CORNER = np.array(
    [dms2deg(38, 44, 19.03, "N"), dms2deg(9, 8, 29.68, "W")]
)  # Latitude and Longitude of top left corner of map
TOP_RIGHT_CORNER = np.array(
    [dms2deg(38, 44, 19.03, "N"), dms2deg(9, 8, 10.05, "W")]
)  # Latitude and Longitude of top right corner of map
BOTTOM_LEFT_CORNER = np.array(
    [dms2deg(38, 44, 5.92, "N"), dms2deg(9, 8, 29.68, "W")]
)  # Latitude and Longitude of bottom left corner of map
BOTTOM_RIGHT_CORNER = np.array(
    [dms2deg(38, 44, 5.92, "N"), dms2deg(9, 8, 10.05, "W")]
)  # Latitude and Longitude of bottom right corner of map

IMAGE_HEIGHT = plt.imread("images/ist.jpeg").shape[0]  # Height of map image in pixels
IMAGE_WIDTH = plt.imread("images/ist.jpeg").shape[1]  # Width of map image in pixels

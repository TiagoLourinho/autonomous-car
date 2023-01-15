import numpy as np
import matplotlib.pyplot as plt

from utils import dms2deg


GMAPS_KEY = "AIzaSyAE3mAAR3DtUpmMY17pS18a7LeSzIbZXFI"  # Google Maps API key


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

IMAGE_HEIGHT, IMAGE_WIDTH = plt.imread("images/ist.jpeg").shape[
    :2
]  # Height/width of map image in pixels

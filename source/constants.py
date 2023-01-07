import numpy as np
import matplotlib.pyplot as plt

ORIGIN = np.array([38.737151, -9.139810]) #np.array([38.737953, -9.138711])        # Latitude and Longitude of entrance of north tower

GMAPS_KEY = "AIzaSyAE3mAAR3DtUpmMY17pS18a7LeSzIbZXFI" # Google Maps API key

""" TOP_LEFT_CORNER = np.array([38.73821111, -9.14114444]) # Latitude and Longitude of top left corner of map
TOP_RIGHT_CORNER = np.array([38.73819444, -9.13629167]) # Latitude and Longitude of top right corner of map
BOTTOM_LEFT_CORNER = np.array([38.73537500, -9.14116667]) # Latitude and Longitude of bottom left corner of map
BOTTOM_RIGHT_CORNER = np.array([38.73535556, -9.13631389]) # Latitude and Longitude of bottom right corner of map """

TOP_LEFT_CORNER = np.array([38.73821111, -9.14116667]) # Latitude and Longitude of top left corner of map
TOP_RIGHT_CORNER = np.array([38.73821111, -9.13631389]) # Latitude and Longitude of top right corner of map
BOTTOM_LEFT_CORNER = np.array([38.73537500, -9.14116667]) # Latitude and Longitude of bottom left corner of map
BOTTOM_RIGHT_CORNER = np.array([38.73537500, -9.13631389]) # Latitude and Longitude of bottom right corner of map

IMAGE_HEIGHT = plt.imread('images/map_improved.png').shape[0] # Height of map image in pixels
IMAGE_WIDTH = plt.imread('images/map_improved.png').shape[1] # Width of map image in pixels
